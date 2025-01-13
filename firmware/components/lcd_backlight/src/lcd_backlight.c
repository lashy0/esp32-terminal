#include <stdlib.h>

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "lcd_backlight.h"

static const char *TAG = "lcd_backlight";

esp_err_t backlight_init(const backlight_config_t *config, backlight_handle_t *handle)
{
    if (!config || !handle) {
        ESP_LOGE(TAG, "Invalid argument: config or handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    const ledc_channel_config_t channel_config = {
        .gpio_num = (gpio_num_t)config->gpio_num,
        .speed_mode = config->leds_mode,
        .channel = config->leds_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = config->leds_timer,
        .duty = 0,
        .hpoint = 0,
        .flags = {
            .output_invert = 0,
        }
    };

    const ledc_timer_config_t timer_config = {
        .speed_mode = config->leds_mode,
        .duty_resolution = config->duty_resolution,
        .timer_num = config->leds_timer,
        .freq_hz = config->freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    handle->channel = config->leds_channel;
    handle->leds_mode = config->leds_mode;
    handle->duty = (1 << config->duty_resolution) - 1;

    return ESP_OK;
}

esp_err_t backlight_set_brightness(const backlight_handle_t *handle, int brightness_percent)
{
    if (!handle) {
        ESP_LOGE(TAG, "Invalid argument: handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (brightness_percent < 0 || brightness_percent > 100) {
        ESP_LOGE(TAG, "Invalid argument: brightness_percent is out of range");
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t duty = (brightness_percent * handle->duty) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(handle->leds_mode, handle->channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(handle->leds_mode, handle->channel));

    return ESP_OK;
}