#pragma once

#include "driver/ledc.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t gpio_num;
    ledc_mode_t leds_mode;
    ledc_channel_t leds_channel;
    ledc_timer_t leds_timer;
    ledc_timer_bit_t duty_resolution;
    uint32_t freq_hz;
} backlight_config_t;

typedef struct {
    ledc_channel_t channel;
    ledc_mode_t leds_mode;
    uint32_t duty;
} backlight_handle_t;

/**
 * @brief Initialize backlight
 * 
 * @param[in] config backlight configuration
 * @param[out] handle backlight handle
 * @return
 *         - ESP_ERR_INVALID_ARG if parameter is invalid
 *         - ESP_OK on success
 */
esp_err_t backlight_init(const backlight_config_t *config, backlight_handle_t *handle);

/**
 * @brief Set backlight brightness
 * 
 * @param[in] handle backlight handle
 * @param[in] brightness_percent brightness in percent
 * @return
 *         - ESP_ERR_INVALID_ARG if parameter is invalid
 *         - ESP_OK on success
 */
esp_err_t backlight_set_brightness(const backlight_handle_t *handle, int brightness_percent);

#ifdef __cplusplus
}
#endif