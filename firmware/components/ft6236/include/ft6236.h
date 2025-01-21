#pragma once

#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_LCD_TOUCH_IO_I2C_FT6236_ADDRESS 0x38

/**
 * @brief Create a new FT6236 touch driver
 * 
 * @note The I2C communication should be initialized before use this function.
 * 
 * @param[in] io LCD/Touch panel IO handle
 * @param[in] config Touch configuration
 * @param[out] out_touch Touch instance handle
 * @return
 *         - ESP_ERR_NO_MEM        if out of memory
 *         - ESP_OK                on success
 */
esp_err_t esp_lcd_touch_new_i2c_ft6236(const esp_lcd_panel_io_handle_t io,
                                       const esp_lcd_touch_config_t *config,
                                       esp_lcd_touch_handle_t *out_touch);

#ifdef __cplusplus
}
#endif