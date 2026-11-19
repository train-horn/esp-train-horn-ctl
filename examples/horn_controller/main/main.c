/*
 * horn_controller - ESP-IDF example for train_horn_ctl.
 *
 * Inputs : horn button (GPIO, active low), pattern button (GPIO, active low),
 *          ignition sense (GPIO via opto/divider, active high),
 *          0.5-4.5 V pressure transducer on ADC1 (through a divider).
 * Outputs: horn solenoid relay, compressor relay (GPIO, active high).
 *
 * A 10 ms esp_timer drives th_ctl_tick(). Pin numbers are examples for an
 * ESP32 DevKitC; change them for your board.
 *
 * SPDX-License-Identifier: MIT
 */
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "train_horn_ctl.h"

#define PIN_HORN_BTN     GPIO_NUM_25
#define PIN_PATTERN_BTN  GPIO_NUM_26
#define PIN_IGNITION     GPIO_NUM_27
#define PIN_SOLENOID     GPIO_NUM_32
#define PIN_COMPRESSOR   GPIO_NUM_33
#define PRESSURE_ADC_CH  ADC_CHANNEL_6 /* GPIO34 on ESP32 */

#define TICK_MS          10
/* Transducer: 0.5 V = 0 psi, 4.5 V = 200 psi, scaled by a divider so that
 * 4.5 V reaches the pin as ~2.9 V (e.g. 10k / 18k). Raw ADC reading is
 * uncalibrated: calibrate against a gauge (see README). */
#define DIVIDER_RATIO    (18.0f / 28.0f)
#define SENSOR_MAX_PSI   200.0f
#define ADC_FULL_MV      3100.0f

static const char *TAG = "horn";

static th_ctl_t s_ctl;
static th_step_t s_fra_steps[TH_FRA_STEPS];
static th_pattern_t s_fra;
static adc_oneshot_unit_handle_t s_adc;
static int64_t s_last_us;
static bool s_prev_pattern_btn;

static float read_tank_psi(void)
{
    int raw = 0;
    if (adc_oneshot_read(s_adc, PRESSURE_ADC_CH, &raw) != ESP_OK) {
        return -100.0f; /* treated as a sensor fault by the interlock */
    }
    float pin_mv = (float)raw * ADC_FULL_MV / 4095.0f;
    float sensor_v = pin_mv / 1000.0f / DIVIDER_RATIO;
    /* below ~0.3 V usually means a broken wire: report it as a fault */
    if (sensor_v < 0.3f) {
        return -100.0f;
    }
    return (sensor_v - 0.5f) * (SENSOR_MAX_PSI / 4.0f);
}

static void tick_cb(void *arg)
{
    (void)arg;
    int64_t now = esp_timer_get_time();
    uint32_t dt_ms = (uint32_t)((now - s_last_us) / 1000);
    if (dt_ms == 0) {
        return;
    }
    s_last_us += (int64_t)dt_ms * 1000;

    bool horn_btn = gpio_get_level(PIN_HORN_BTN) == 0;
    bool pattern_btn = gpio_get_level(PIN_PATTERN_BTN) == 0;
    bool ignition = gpio_get_level(PIN_IGNITION) == 1;

    /* Pattern button: press starts the crossing pattern, press again stops. */
    if (pattern_btn && !s_prev_pattern_btn) {
        if (th_seq_running(&s_ctl.seq)) {
            th_seq_stop(&s_ctl.seq);
        } else {
            th_seq_start(&s_ctl.seq, &s_fra);
        }
    }
    s_prev_pattern_btn = pattern_btn;
    th_ctl_set_manual(&s_ctl, horn_btn);

    th_ctl_out_t out = th_ctl_tick(&s_ctl, dt_ms, read_tank_psi(), ignition);
    gpio_set_level(PIN_SOLENOID, out.solenoid ? 1 : 0);
    gpio_set_level(PIN_COMPRESSOR, out.compressor ? 1 : 0);
}

void app_main(void)
{
    /* Outputs first, driven low, so relays stay off during boot. */
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << PIN_SOLENOID) | (1ULL << PIN_COMPRESSOR),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_set_level(PIN_SOLENOID, 0));
    ESP_ERROR_CHECK(gpio_set_level(PIN_COMPRESSOR, 0));
    ESP_ERROR_CHECK(gpio_config(&out_cfg));

    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << PIN_HORN_BTN) | (1ULL << PIN_PATTERN_BTN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in_cfg));
    gpio_config_t ign_cfg = {
        .pin_bit_mask = 1ULL << PIN_IGNITION,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&ign_cfg));

    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));
    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, PRESSURE_ADC_CH, &ch_cfg));

    /* 25 % long-run duty, at most 8 s continuous: protects the valve coil
     * and keeps the compressor from running flat out. */
    th_duty_cfg_t duty = { .max_duty_permille = 250, .max_on_ms = 8000 };
    th_interlock_cfg_t il;
    th_interlock_cfg_default(&il); /* horn 90/100 psi, compressor 120/150 psi */
    il.sensor_max_psi = SENSOR_MAX_PSI + 5.0f;
    if (!th_ctl_init(&s_ctl, &duty, &il)) {
        ESP_LOGE(TAG, "bad configuration");
        return;
    }
    th_pattern_fra_default(s_fra_steps, false, &s_fra);

    const esp_timer_create_args_t targs = {
        .callback = tick_cb,
        .name = "horn_tick",
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&targs, &timer));
    s_last_us = esp_timer_get_time();
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, TICK_MS * 1000));

    ESP_LOGI(TAG, "train_horn_ctl %s running", TRAIN_HORN_CTL_VERSION);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG, "duty available %u ms, locked=%d",
                 (unsigned)th_duty_available_ms(&s_ctl.duty),
                 th_duty_locked(&s_ctl.duty));
    }
}
