#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "lvgl.h"

#include "ili9488.h"
#include "lcd_backlight.h"
#include "ft6236.h"

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

#define I2C_MASTER_SCL_IO           1
#define I2C_MASTER_SDA_IO           2
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

static const size_t LV_BUFFER_SIZE = LCD_H_RES * 25;
static const int LVGL_UPDATE_PERIOD_MS = 5;

static backlight_handle_t bk_handle;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;

static esp_lcd_touch_handle_t tp = NULL;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *lv_buf_1 = NULL;
static lv_color_t *lv_buf_2 = NULL;

static lv_disp_t *disp = NULL;
static lv_disp_drv_t disp_drv;

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    esp_lcd_panel_draw_bitmap(panel, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void lvgl_touch_cb(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Read touch controller data */
    esp_lcd_touch_read_data(drv->user_data);

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;

        ESP_LOGI(TAG, "Touch at X: %d, Y: %d", data->point.x, data->point.y);
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void lvgl_tick_cb(void *arg)
{
    lv_tick_inc(LVGL_UPDATE_PERIOD_MS);
}

static void event_handler(lv_event_t * e)
{
    lv_obj_t * obj = lv_event_get_target(e);
    LV_LOG_USER("Button %d clicked", (int)lv_obj_get_index(obj));
}

void create_ui(void)
{
    lv_obj_t *scr = lv_scr_act();

    /*Create a Tab view object*/
    lv_obj_t * tabview;
    tabview = lv_tabview_create(scr, LV_DIR_TOP, 50);

    /*Add 3 tabs (the tabs are page (lv_page) and can be scrolled*/
    lv_obj_t * tab1 = lv_tabview_add_tab(tabview, "Tab 1");
    lv_obj_t * tab2 = lv_tabview_add_tab(tabview, "Tab 2");
    lv_obj_t * tab3 = lv_tabview_add_tab(tabview, "Tab 3");

    /*Add content to the tabs*/
    lv_obj_t * label = lv_label_create(tab1);
    lv_label_set_text(label, "This the first tab\n\n"
                      "If the content\n"
                      "of a tab\n"
                      "becomes too\n"
                      "longer\n"
                      "than the\n"
                      "container\n"
                      "then it\n"
                      "automatically\n"
                      "becomes\n"
                      "scrollable.\n"
                      "\n"
                      "\n"
                      "\n"
                      "Can you see it?");

    label = lv_label_create(tab2);
    lv_label_set_text(label, "Second tab");

    label = lv_label_create(tab3);
    lv_label_set_text(label, "Third tab");

    lv_obj_scroll_to_view_recursive(label, LV_ANIM_ON);
}

void app_main(void)
{
    // Initialize I2C bus
    ESP_LOGI(TAG, "Initialize I2C bus...");
    i2c_master_bus_handle_t i2c_bus_handle = NULL;

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = {
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_FT6236_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .control_phase_bytes = 1,
        .dc_bit_offset = 0,
        .lcd_cmd_bits = 8,
        .flags = {
            .disable_control_phase = 1,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_handle, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 1,
            .mirror_x = 0,
            .mirror_y = 1,
        },
    };

    ESP_LOGI(TAG, "Initialize touch controller FT6236");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_ft6236(tp_io_handle, &tp_cfg, &tp));

    ESP_LOGI(TAG, "Initializing LCD backlight...");
    backlight_config_t bk_config = {
        .gpio_num = LCD_PIN_NUM_BL,
        .leds_mode = LEDC_LOW_SPEED_MODE,
        .leds_channel = LEDC_CHANNEL_0,
        .leds_timer = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 5000,
    };

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
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_PIN_NUM_DC,
        .cs_gpio_num = LCD_PIN_NUM_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
        .flags = {
            .dc_low_on_data = 0,
            .octal_mode = 0,
            .sio_mode = 0,
            .lsb_first = 0,
            .cs_high_active = 0,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io_handle));

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 18,
        .flags = {
            .reset_active_high = 0,
        },
        .vendor_config = NULL
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9488(io_handle, &panel_cfg, LV_BUFFER_SIZE, &panel_handle));

    // The portrait format is now available

    // to get the landscape format
    // swap_xy -> true
    // mirror -> fasle true
    // hor_res -> LCD_V_RES
    // ver_res -> LCD_H_RES

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Initializing LVGL...");
    lv_init();
    ESP_LOGI(TAG, "Allocating %zu bytes for LVGL buffer", LV_BUFFER_SIZE * sizeof(lv_color_t));
    lv_buf_1 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    ESP_LOGI(TAG, "Allocating %zu bytes for second LVGL buffer", LV_BUFFER_SIZE * sizeof(lv_color_t));
    lv_buf_2 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    ESP_LOGI(TAG, "Creating LVGL display buffer...");
    lv_disp_draw_buf_init(&draw_buf, lv_buf_1, lv_buf_2, LV_BUFFER_SIZE);

    // Как сделать настройку ориентации дисплея
    ESP_LOGI(TAG, "Initializing %dx%d display", LCD_H_RES, LCD_V_RES);
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_V_RES;
    disp_drv.ver_res = LCD_H_RES;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.user_data = panel_handle;
    disp = lv_disp_drv_register(&disp_drv);

    if (!disp) {
        ESP_LOGE(TAG, "Failed to register display driver");
        return;
    }

    ESP_LOGI(TAG, "Initailize touch driver");
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = lvgl_touch_cb;
    indev_drv.user_data = tp;

    lv_indev_drv_register(&indev_drv);

    ESP_LOGI(TAG, "Setting up LVGL tick timer...");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_tick_cb,
        .name = "lvgl_tick_timer",
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_UPDATE_PERIOD_MS * 1000));

    // gpio_set_direction(LCD_PIN_NUM_BL, GPIO_MODE_OUTPUT);
    // gpio_set_level(LCD_PIN_NUM_BL, 1);

    ESP_ERROR_CHECK(backlight_set_brightness(&bk_handle, 100));

    // UI LVGL
    create_ui();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_timer_handler();
    }
}