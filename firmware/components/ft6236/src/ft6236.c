#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_lcd_panel_io.h"

#include "esp_lcd_touch.h"

static const char *TAG = "lcd_touch_ft6236";

// FT6236 Register Definitions
#define FT6236_DEVICE_MODE              0x00
#define FT6236_GESTURE_ID               0x01
#define FT6236_TOUCH_POINTS             0x02

#define FT6236_TOUCH1_XH                0x03
#define FT6236_TOUCH1_XL                0x04
#define FT6236_TOUCH1_YH                0x05
#define FT6236_TOUCH1_YL                0x06

#define FT6236_TOUCH2_XH                0x09
#define FT6236_TOUCH2_XL                0x0A
#define FT6236_TOUCH2_YH                0x0B
#define FT6236_TOUCH2_YL                0x0C

#define FT6236_PANEL_ID                 0xA8
#define FT6236_TH_GROUP                 0x80
#define FT6236_CTRL                     0x86
#define FT6236_TOUCHRATE_ACTIVE         0x88
#define FT6236_TH_DIFF                  0x85

static esp_err_t esp_lcd_touch_ft6236_read_data(esp_lcd_touch_handle_t tp);
static bool esp_lcd_touch_ft6236_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num);
static esp_err_t esp_lcd_touch_ft6236_del(esp_lcd_touch_handle_t tp);

static esp_err_t touch_ft6236_i2c_write(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t data);
static esp_err_t touch_ft6236_i2c_read(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t *data, uint8_t len);

static esp_err_t touch_ft6236_init(esp_lcd_touch_handle_t tp);
static esp_err_t touch_ft6236_reset(esp_lcd_touch_handle_t tp);

esp_err_t esp_lcd_touch_new_i2c_ft6236(const esp_lcd_panel_io_handle_t io,
                                       const esp_lcd_touch_config_t *config,
                                       esp_lcd_touch_handle_t *out_touch)
{
    esp_err_t ret = ESP_OK;

    assert(config != NULL);
    assert(out_touch != NULL);

     /* Prepare main structure */
    esp_lcd_touch_handle_t esp_lcd_touch_ft6236 = heap_caps_calloc(1, sizeof(esp_lcd_touch_t), MALLOC_CAP_DEFAULT);
    ESP_GOTO_ON_FALSE(esp_lcd_touch_ft6236, ESP_ERR_NO_MEM, err, TAG, "no mem for FT6236 controller");

    /* Communication interface */
    esp_lcd_touch_ft6236->io = io;

     /* Only supported callbacks are set */
    esp_lcd_touch_ft6236->read_data = esp_lcd_touch_ft6236_read_data;
    esp_lcd_touch_ft6236->get_xy = esp_lcd_touch_ft6236_get_xy;
    esp_lcd_touch_ft6236->del = esp_lcd_touch_ft6236_del;

     /* Mutex */
    esp_lcd_touch_ft6236->data.lock.owner = portMUX_FREE_VAL;

    /* Save config */
    memcpy(&esp_lcd_touch_ft6236->config, config, sizeof(esp_lcd_touch_config_t));

    /* Prepare pin for touch interrupt */
    if (esp_lcd_touch_ft6236->config.int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = (esp_lcd_touch_ft6236->config.levels.interrupt ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE),
            .pin_bit_mask = BIT64(esp_lcd_touch_ft6236->config.int_gpio_num)
        };
        ret = gpio_config(&int_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");

        /* Register interrupt callback */
        if (esp_lcd_touch_ft6236->config.interrupt_callback) {
            esp_lcd_touch_register_interrupt_callback(esp_lcd_touch_ft6236, esp_lcd_touch_ft6236->config.interrupt_callback);
        }
    }

    /* Prepare pin for touch controller reset */
    if (esp_lcd_touch_ft6236->config.rst_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t rst_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = BIT64(esp_lcd_touch_ft6236->config.rst_gpio_num)
        };
        ret = gpio_config(&rst_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");
    }

    /* Reset controller */
    ret = touch_ft6236_reset(esp_lcd_touch_ft6236);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "FT6236 reset failed");

    /* Init controller */
    ret = touch_ft6236_init(esp_lcd_touch_ft6236);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "FT6236 init failed");

err:
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error (0x%x)! Touch controller FT6236 initialization failed!", ret);
        if (esp_lcd_touch_ft6236) {
            esp_lcd_touch_ft6236_del(esp_lcd_touch_ft6236);
        }
    }

    *out_touch = esp_lcd_touch_ft6236;

    return ret;
}

static esp_err_t esp_lcd_touch_ft6236_read_data(esp_lcd_touch_handle_t tp)
{
    esp_err_t err;
    uint8_t data[30];
    uint8_t points;
    size_t i = 0;

    assert(tp != NULL);

    err = touch_ft6236_i2c_read(tp, FT6236_TOUCH_POINTS, &points, 1);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    if (points > 5 || points == 0) {
        return ESP_OK;
    }

    /* Number of touched points */
    points = (points > CONFIG_ESP_LCD_TOUCH_MAX_POINTS ? CONFIG_ESP_LCD_TOUCH_MAX_POINTS : points);

    err = touch_ft6236_i2c_read(tp, FT6236_TOUCH1_XH, data, 6 * points);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    portENTER_CRITICAL(&tp->data.lock);

    /* Number of touched points */
    tp->data.points = points;

    /* Fill all coordinates */
    for (i = 0; i < points; i++) {
        tp->data.coords[i].x = (((uint16_t)data[(i * 6) + 0] & 0x0f) << 8) + data[(i * 6) + 1];
        tp->data.coords[i].y = (((uint16_t)data[(i * 6) + 2] & 0x0f) << 8) + data[(i * 6) + 3];
    }

    portEXIT_CRITICAL(&tp->data.lock);

    return ESP_OK;
}

static bool esp_lcd_touch_ft6236_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    assert(tp != NULL);
    assert(x != NULL);
    assert(y != NULL);
    assert(point_num != NULL);
    assert(max_point_num > 0);

    portENTER_CRITICAL(&tp->data.lock);

    /* Count of points */
    *point_num = (tp->data.points > max_point_num ? max_point_num : tp->data.points);

    for (size_t i = 0; i < *point_num; i++) {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;

        if (strength) {
            strength[i] = tp->data.coords[i].strength;
        }
    }

    /* Invalidate */
    tp->data.points = 0;

    portEXIT_CRITICAL(&tp->data.lock);

    return (*point_num > 0);
}

static esp_err_t esp_lcd_touch_ft6236_del(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    /* Reset GPIO pin settings */
    if (tp->config.int_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.int_gpio_num);
        if (tp->config.interrupt_callback) {
            gpio_isr_handler_remove(tp->config.int_gpio_num);
        }
    }

    /* Reset GPIO pin settings */
    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }

    free(tp);

    return ESP_OK;
}

static esp_err_t touch_ft6236_i2c_read(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t *data, uint8_t len)
{
    assert(tp != NULL);
    assert(data != NULL);

    /* Read data */
    return esp_lcd_panel_io_rx_param(tp->io, reg, data, len);
}

static esp_err_t touch_ft6236_i2c_write(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t data)
{
    assert(tp != NULL);

    // *INDENT-OFF*
    /* Write data */
    return esp_lcd_panel_io_tx_param(tp->io, reg, (uint8_t[]){data}, 1);
    // *INDENT-ON*
}

static esp_err_t touch_ft6236_init(esp_lcd_touch_handle_t tp)
{
    esp_err_t ret = ESP_OK;
    uint8_t reg_val;

    ret = touch_ft6236_i2c_read(tp, FT6236_PANEL_ID, &reg_val, 1);
    ESP_LOGI(TAG, "FT6236 detected with ID: 0x%02X", reg_val);

    ret = touch_ft6236_i2c_write(tp, FT6236_DEVICE_MODE, 0x00);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set normal operation mode");

    ret = touch_ft6236_i2c_write(tp, FT6236_TH_GROUP, 0x20);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failde to set touch threshold");

    ret = touch_ft6236_i2c_write(tp, FT6236_CTRL, 0x01);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set auto-monitor mode");

    ret = touch_ft6236_i2c_write(tp, FT6236_TOUCHRATE_ACTIVE, 0x0A);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set active mode report rate");

    ret = touch_ft6236_i2c_write(tp, FT6236_TH_DIFF, 0x04);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to set jitter filter");

    return ret;
}

static esp_err_t touch_ft6236_reset(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, tp->config.levels.reset), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, !tp->config.levels.reset), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_OK;
}