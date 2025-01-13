#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

#include "ili9488.h"
#include "lcd_backlight.h"

static const char *TAG = "app_main";

#define LCD_PIN_NUM_MOSI    13
#define LCD_PIN_NUM_MISO    14
#define LCD_PIN_NUM_SCLK    12
#define LCD_PIN_NUM_CS      3
#define LCD_PIN_NUM_DC      42
#define LCD_PIN_NUM_RST     -1
#define LCD_PIN_NUM_BL      46

#define LCD_HOST            SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ  (40 * 1000 * 1000)
#define LCD_H_RES           320
#define LCD_V_RES           480

#define LCD_CMD_BITS        8
#define LCD_PARAM_BITS      8

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing LCD backlight...");
    backlight_config_t bk_config = {
        .gpio_num = LCD_PIN_NUM_BL,
        .leds_mode = LEDC_LOW_SPEED_MODE,
        .leds_channel = LEDC_CHANNEL_0,
        .leds_timer = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 5000,
    };

    backlight_handle_t bk_handle;
    ESP_ERROR_CHECK(backlight_init(&bk_config, &bk_handle));
    ESP_ERROR_CHECK(backlight_set_brightness(&bk_handle, 0));

    // ESP_LOGI(TAG, "Turn off LCD backlight...");
    // gpio_config_t bk_gpio_cong = {
    //     .mode = GPIO_MODE_OUTPUT,
    //     .pin_bit_mask = 1ULL << LCD_PIN_NUM_BL,
    // };
    // ESP_ERROR_CHECK(gpio_config(&bk_gpio_cong));

    ESP_LOGI(TAG, "Initializing LCD bus...");
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_PIN_NUM_SCLK,
        .mosi_io_num = LCD_PIN_NUM_MOSI,
        .miso_io_num = LCD_PIN_NUM_MISO,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 32768,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Initializing panel IO...");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_PIN_NUM_DC,
        .cs_gpio_num = LCD_PIN_NUM_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .flags = {
            .dc_low_on_data = 0,
            .octal_mode = 0,
            .sio_mode = 0,
            .lsb_first = 0,
            .cs_high_active = 0,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 18,
        .flags = {
            .reset_active_high = 0,
        },
        .vendor_config = NULL
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9488(io_handle, &panel_cfg, LCD_H_RES * 25, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // gpio_set_direction(LCD_PIN_NUM_BL, GPIO_MODE_OUTPUT);
    // gpio_set_level(LCD_PIN_NUM_BL, 1);

    ESP_ERROR_CHECK(backlight_set_brightness(&bk_handle, 100));
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(backlight_set_brightness(&bk_handle, 50));
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_ERROR_CHECK(backlight_set_brightness(&bk_handle, 100));

    uint16_t color = 0xF800; // Красный

    size_t buffer_size = 50 * 50;
    uint16_t *buffer = (uint16_t *)malloc(buffer_size * sizeof(uint16_t));
    if (!buffer) {
        printf("Error: Not enough memory for screen buffer\n");
        return;
    }
    for (size_t i = 0; i < buffer_size; i++) {
        buffer[i] = color;
    }

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 50, 50, buffer));

    free(buffer);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}