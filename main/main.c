/*
 * BridgeSense - detect a person bridging two metal plates via a resistive divider
 *
 * Hardware (ESP32-S3 N16R8):
 *
 *   3V3 --[R_TOP 1M]--+--[R_BOTTOM 1M]-- GND
 *                     |
 *                     +--[100nF]-- GND         (noise filter / ADC sample-and-hold buffer)
 *                     +--[100k-470k]-- PLATE A (large plate, series resistor = ESD protection)
 *                     +-- ADC pin (GPIO4 = ADC1_CH3)
 *
 *   PLATE B -- GND
 *
 * When a finger/hand touches both plates, body resistance appears in parallel
 * with R_BOTTOM and the ADC voltage drops.
 *
 * Tested API level: ESP-IDF v5.x (adc_oneshot driver).
 * Note: GPIO4 is ADC1_CH3 on ESP32-S3. Avoid ADC2 pins if you also use Wi-Fi.
 */

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "bridge_sense";

/* ---------------- Configuration ---------------- */
#define SENSE_UNIT          ADC_UNIT_1
#define SENSE_CHANNEL       ADC_CHANNEL_3      /* GPIO4 on ESP32-S3 */
#define SENSE_ATTEN         ADC_ATTEN_DB_12    /* ~0..3.1 V range */

#define SAMPLES_PER_READ    32                 /* oversampling for noise rejection */
#define LOOP_PERIOD_MS      50                 /* RC tau is ~50-100 ms, so 50 ms is plenty */

#define CALIBRATION_READS   40                 /* ~2 s of idle readings at startup */

#define TOUCH_DROP_RATIO    0.80f              /* touched if mV < baseline * 0.80 (20% drop) */
#define RELEASE_DROP_RATIO  0.90f              /* released if mV > baseline * 0.90 (hysteresis) */
#define DEBOUNCE_COUNT      3                  /* consecutive readings needed to change state */

#define BASELINE_ALPHA      0.01f              /* slow drift tracking, only while untouched */
#define MIN_BASELINE_MV     200                /* sanity: below this, wiring is probably wrong */

/* ---------------- Globals ---------------- */
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static bool s_cali_ok = false;

/* ---------------- ADC helpers ---------------- */
static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = SENSE_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = SENSE_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, SENSE_CHANNEL, &chan_cfg));

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = SENSE_UNIT,
        .chan = SENSE_CHANNEL,
        .atten = SENSE_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali) == ESP_OK) {
        s_cali_ok = true;
        ESP_LOGI(TAG, "ADC calibration: curve fitting");
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = SENSE_UNIT,
        .atten = SENSE_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_cali) == ESP_OK) {
        s_cali_ok = true;
        ESP_LOGI(TAG, "ADC calibration: line fitting");
    }
#endif
    if (!s_cali_ok) {
        ESP_LOGW(TAG, "ADC calibration unavailable, using raw-scaled values");
    }
}

/* Returns the averaged reading in millivolts. */
static int read_mv(void)
{
    int64_t sum = 0;
    int valid = 0;
    for (int i = 0; i < SAMPLES_PER_READ; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc, SENSE_CHANNEL, &raw) == ESP_OK) {
            sum += raw;
            valid++;
        }
    }
    if (valid == 0) {
        return -1;
    }
    int raw_avg = (int)(sum / valid);

    if (s_cali_ok) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(s_cali, raw_avg, &mv) == ESP_OK) {
            return mv;
        }
    }
    /* Fallback: approximate scaling for 12 dB attenuation, 12-bit */
    return (raw_avg * 3100) / 4095;
}

/* Average several idle readings to obtain the no-touch baseline. */
static int calibrate_baseline(void)
{
    int64_t sum = 0;
    int n = 0;
    for (int i = 0; i < CALIBRATION_READS; i++) {
        int mv = read_mv();
        if (mv >= 0) {
            sum += mv;
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
    return n ? (int)(sum / n) : -1;
}

/* ---------------- Main ---------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "BridgeSense starting. Keep both plates untouched for calibration...");
    adc_init();

    /* Let the RC filter settle before sampling the baseline. */
    vTaskDelay(pdMS_TO_TICKS(500));

    int baseline_mv = calibrate_baseline();
    if (baseline_mv < MIN_BASELINE_MV) {
        ESP_LOGE(TAG, "Baseline %d mV is too low. Check wiring (R_top to 3V3, plate not shorted).",
                 baseline_mv);
    }
    ESP_LOGI(TAG, "Baseline = %d mV (touch < %d mV, release > %d mV)",
             baseline_mv,
             (int)(baseline_mv * TOUCH_DROP_RATIO),
             (int)(baseline_mv * RELEASE_DROP_RATIO));

    float baseline = (float)baseline_mv;
    bool touched = false;
    int streak = 0;

    while (1) {
        int mv = read_mv();
        if (mv < 0) {
            ESP_LOGW(TAG, "ADC read failed");
            vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
            continue;
        }

        /* Decide what state this single reading suggests. */
        bool want_touched = touched;
        if (!touched && mv < baseline * TOUCH_DROP_RATIO) {
            want_touched = true;
        } else if (touched && mv > baseline * RELEASE_DROP_RATIO) {
            want_touched = false;
        }

        /* Debounce: require N consecutive readings that disagree with current state. */
        if (want_touched != touched) {
            if (++streak >= DEBOUNCE_COUNT) {
                touched = want_touched;
                streak = 0;
                if (touched) {
                    ESP_LOGI(TAG, ">>> CIRCUIT COMPLETE (person bridging plates), %d mV, baseline %.0f mV",
                             mv, baseline);
                } else {
                    ESP_LOGI(TAG, "<<< CIRCUIT OPEN (released), %d mV", mv);
                }
            }
        } else {
            streak = 0;
        }

        /* Track slow drift (humidity, temperature) only while idle. */
        if (!touched && streak == 0) {
            baseline += BASELINE_ALPHA * ((float)mv - baseline);
        }

        /* Raw stream for tuning; remove or lower log level once happy. */
        ESP_LOGD(TAG, "mv=%d baseline=%.0f state=%s", mv, baseline, touched ? "TOUCH" : "idle");

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}