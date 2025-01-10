#pragma once

#include "esp_lcd_panel_vendor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create LCD panel for model ILI9488
 *
 * @param[in] io LCD panel IO handle
 * @param[in] panel_dev_config general panel device configuration
 * @param[in] buffer_size size of the buffer to allocate for color convertion
 * @param[out] ret_panel returned LCD panel handle
 * @return
 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
 *         - ESP_ERR_NO_MEM        if out of memory
 *         - ESP_OK                on success
 */
esp_err_t esp_lcd_new_panel_ili9488(const esp_lcd_panel_io_handle_t io,
                                    const esp_lcd_panel_dev_config_t *panel_dev_config,
                                    const size_t buffer_size,
                                    esp_lcd_panel_handle_t *ret_panel);

#ifdef __cplusplus
}
#endif