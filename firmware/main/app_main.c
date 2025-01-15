#include <stdio.h>

#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "lvgl.h"

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

static const size_t LV_BUFFER_SIZE = LCD_H_RES * 25;
static const int LVGL_UPDATE_PERIOD_MS = 5;

static backlight_handle_t bk_handle;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;

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

static void lvgl_tick_cb(void *arg)
{
    lv_tick_inc(LVGL_UPDATE_PERIOD_MS);
}

lv_obj_t *value_temp = NULL;
lv_obj_t *value_humid = NULL;
lv_obj_t *value_pressure = NULL;
lv_obj_t *value_co2 = NULL;
lv_obj_t *value_voc = NULL;
lv_obj_t *value_iaq = NULL;

void create_ui(void)
{
    lv_obj_t *scr = lv_scr_act();

    //

    // Style frame indicator temperature and humidity
    static lv_style_t style_frame_indic_temp_humid;
    lv_style_init(&style_frame_indic_temp_humid);
    lv_style_set_radius(&style_frame_indic_temp_humid, 15);
    lv_style_set_bg_color(&style_frame_indic_temp_humid, lv_color_black());
    lv_style_set_bg_opa(&style_frame_indic_temp_humid, LV_OPA_COVER);
    lv_style_set_pad_all(&style_frame_indic_temp_humid, 10);

    // Create frame indicator temperature and humidity
    lv_obj_t *frame_indic_temp_humid = lv_obj_create(scr);
    lv_obj_set_size(frame_indic_temp_humid, 310, 92);
    lv_obj_align(frame_indic_temp_humid, LV_ALIGN_TOP_LEFT, 15, 43);
    lv_obj_add_style(frame_indic_temp_humid, &style_frame_indic_temp_humid, 0);
    lv_obj_clear_flag(frame_indic_temp_humid, LV_OBJ_FLAG_SCROLLABLE);

    // Style label temperature and humidity
    static lv_style_t style_text_temp_humidity_label;
    lv_style_init(&style_text_temp_humidity_label);
    lv_style_set_text_color(&style_text_temp_humidity_label, lv_color_white());
    lv_style_set_text_font(&style_text_temp_humidity_label, &lv_font_montserrat_14);

    // Style value temperature and humidity
    static lv_style_t style_value_temp_humidity;
    lv_style_init(&style_value_temp_humidity);
    lv_style_set_text_color(&style_value_temp_humidity, lv_color_white());
    lv_style_set_text_font(&style_value_temp_humidity, &lv_font_montserrat_40);

    // Create label temperature
    lv_obj_t *label_temp = lv_label_create(frame_indic_temp_humid);
    lv_label_set_text(label_temp, "Temperature:");
    lv_obj_add_style(label_temp, &style_text_temp_humidity_label, 0);
    lv_obj_align(label_temp, LV_ALIGN_TOP_LEFT, 0, 0);

    // Create value temperature
    value_temp = lv_label_create(frame_indic_temp_humid);
    lv_label_set_text(value_temp, "27°C");
    lv_obj_add_style(value_temp, &style_value_temp_humidity, 0);
    lv_obj_align(value_temp, LV_ALIGN_TOP_LEFT, 0, 25);

    // Create label Humidity
    lv_obj_t *label_humid = lv_label_create(frame_indic_temp_humid);
    lv_label_set_text(label_humid, "Humidity:");
    lv_obj_add_style(label_humid, &style_text_temp_humidity_label, 0);
    lv_obj_align(label_humid, LV_ALIGN_TOP_LEFT, 160, 0);

    // Create value Humidity
    value_humid = lv_label_create(frame_indic_temp_humid);
    lv_label_set_text(value_humid, "35%");
    lv_obj_add_style(value_humid, &style_value_temp_humidity, 0);
    lv_obj_align(value_humid, LV_ALIGN_TOP_LEFT, 160, 25);

    //

    // Style frame indicator voc
    static lv_style_t style_frame_voc;
    lv_style_init(&style_frame_voc);
    lv_style_set_radius(&style_frame_voc, 15);
    lv_style_set_bg_color(&style_frame_voc, lv_color_black());
    lv_style_set_bg_opa(&style_frame_voc, LV_OPA_COVER);
    lv_style_set_pad_all(&style_frame_voc, 10);

    // Create frame indicator voc
    lv_obj_t *frame_voc = lv_obj_create(scr);
    lv_obj_set_size(frame_voc, 310, 75);
    lv_obj_align(frame_voc, LV_ALIGN_TOP_LEFT, 15, 145);
    lv_obj_add_style(frame_voc, &style_frame_voc, 0);
    lv_obj_clear_flag(frame_voc, LV_OBJ_FLAG_SCROLLABLE);

    // Style label voc
    static lv_style_t style_text_voc_label;
    lv_style_init(&style_text_voc_label);
    lv_style_set_text_color(&style_text_voc_label, lv_color_white());
    lv_style_set_text_font(&style_text_voc_label, &lv_font_montserrat_14);

    // Style value voc
    static lv_style_t style_value_voc;
    lv_style_init(&style_value_voc);
    lv_style_set_text_color(&style_value_voc, lv_color_white());
    lv_style_set_text_font(&style_value_voc, &lv_font_montserrat_26);

    // Create label voc
    lv_obj_t *label_voc = lv_label_create(frame_voc);
    lv_label_set_text(label_voc, "Voc:");
    lv_obj_add_style(label_voc, &style_text_voc_label, 0);
    lv_obj_align(label_voc, LV_ALIGN_TOP_LEFT, 0, 0);

    // Create value voc
    value_voc = lv_label_create(frame_voc);
    lv_label_set_text(value_voc, "0 ppb");
    lv_obj_add_style(value_voc, &style_value_voc, 0);
    lv_obj_align(value_voc, LV_ALIGN_TOP_LEFT, 0, 25);

    //

    // Style frame indicator iaq
    static lv_style_t style_frame_iaq;
    lv_style_init(&style_frame_iaq);
    lv_style_set_radius(&style_frame_iaq, 15);
    lv_style_set_bg_color(&style_frame_iaq, lv_color_black());
    lv_style_set_bg_opa(&style_frame_iaq, LV_OPA_COVER);
    lv_style_set_pad_all(&style_frame_iaq, 10);

    // Create frame indicator iaq
    lv_obj_t *frame_iaq = lv_obj_create(scr);
    lv_obj_set_size(frame_iaq, 310, 75);
    lv_obj_align(frame_iaq, LV_ALIGN_TOP_LEFT, 15, 230);
    lv_obj_add_style(frame_iaq, &style_frame_iaq, 0);
    lv_obj_clear_flag(frame_iaq, LV_OBJ_FLAG_SCROLLABLE);

    // Style label iaq
    static lv_style_t style_text_iaq_label;
    lv_style_init(&style_text_iaq_label);
    lv_style_set_text_color(&style_text_iaq_label, lv_color_white());
    lv_style_set_text_font(&style_text_iaq_label, &lv_font_montserrat_14);

    // Style value iaq
    static lv_style_t style_value_iaq;
    lv_style_init(&style_value_iaq);
    lv_style_set_text_color(&style_value_iaq, lv_color_white());
    lv_style_set_text_font(&style_value_iaq, &lv_font_montserrat_26);

    // Create label iaq
    lv_obj_t *label_iaq = lv_label_create(frame_iaq);
    lv_label_set_text(label_iaq, "Iaq:");
    lv_obj_add_style(label_iaq, &style_text_iaq_label, 0);
    lv_obj_align(label_iaq, LV_ALIGN_TOP_LEFT, 0, 0);

    // Create value iaq
    value_iaq = lv_label_create(frame_iaq);
    lv_label_set_text(value_iaq, "0");
    lv_obj_add_style(value_iaq, &style_value_iaq, 0);
    lv_obj_align(value_iaq, LV_ALIGN_TOP_LEFT, 0, 25);

    //

    // Style frame indicator pressure
    static lv_style_t style_frame_pressure;
    lv_style_init(&style_frame_pressure);
    lv_style_set_radius(&style_frame_pressure, 15);
    lv_style_set_bg_color(&style_frame_pressure, lv_color_black());
    lv_style_set_bg_opa(&style_frame_pressure, LV_OPA_COVER);

    // Create frame indicator pressure
    lv_obj_t *frame_indic_pressure = lv_obj_create(scr);
    lv_obj_set_size(frame_indic_pressure, 125, 125);
    lv_obj_align(frame_indic_pressure, LV_ALIGN_TOP_RIGHT, -15, 43);
    lv_obj_add_style(frame_indic_pressure, &style_frame_pressure, 0);
    lv_obj_clear_flag(frame_indic_pressure, LV_OBJ_FLAG_SCROLLABLE);

    // Style label pressure
    static lv_style_t style_text_value_pressure;
    lv_style_init(&style_text_value_pressure);
    lv_style_set_text_color(&style_text_value_pressure, lv_color_white());
    lv_style_set_text_font(&style_text_value_pressure, &lv_font_montserrat_14);

    // Create label pressure
    lv_obj_t *label_pressure = lv_label_create(frame_indic_pressure);
    lv_label_set_text(label_pressure, "Pressure:");
    lv_obj_add_style(label_pressure, &style_text_value_pressure, 0);
    lv_obj_align(label_pressure, LV_ALIGN_TOP_LEFT, 0, 0);

    value_pressure = lv_label_create(frame_indic_pressure);
    lv_label_set_text(value_pressure, "746 mmHg");
    lv_obj_add_style(value_pressure, &style_text_value_pressure, 0);
    lv_obj_align(value_pressure, LV_ALIGN_TOP_LEFT, 0, 25);

    //

    // Style frame indicator co2
    static lv_style_t style_frame_co2;
    lv_style_init(&style_frame_co2);
    lv_style_set_radius(&style_frame_co2, 15);
    lv_style_set_bg_color(&style_frame_co2, lv_color_black());
    lv_style_set_bg_opa(&style_frame_co2, LV_OPA_COVER);

    // Create frame indicator co2
    lv_obj_t *frame_indic_co2 = lv_obj_create(scr);
    lv_obj_set_size(frame_indic_co2, 125, 125);
    lv_obj_align(frame_indic_co2, LV_ALIGN_TOP_RIGHT, -15, 179);
    lv_obj_add_style(frame_indic_co2, &style_frame_co2, 0);
    lv_obj_clear_flag(frame_indic_co2, LV_OBJ_FLAG_SCROLLABLE);

    // Style label co2
    static lv_style_t style_text_value_co2;
    lv_style_init(&style_text_value_co2);
    lv_style_set_text_color(&style_text_value_co2, lv_color_white());
    lv_style_set_text_font(&style_text_value_co2, &lv_font_montserrat_14);

    // Create label co2
    lv_obj_t *label_co2 = lv_label_create(frame_indic_co2);
    lv_label_set_text(label_co2, "CO2:");
    lv_obj_add_style(label_co2, &style_text_value_co2, 0);
    lv_obj_align(label_co2, LV_ALIGN_TOP_LEFT, 0, 0);

    // Create value co2
    value_co2 = lv_label_create(frame_indic_co2);
    lv_label_set_text(value_co2, "0 ppm");
    lv_obj_add_style(value_co2, &style_text_value_co2, 0);
    lv_obj_align(value_co2, LV_ALIGN_TOP_LEFT, 0, 25);
}

// BLE

static const char *TAG_BLE = "ble_client";

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define REMOTE_SERVICE_UUID 0x181A

static char remote_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "Test [6CAB]";
static bool connect = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
    },
};

static void esp_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_callback(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = REMOTE_SERVICE_UUID,
    },
};

static esp_bt_uuid_t temperature_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = 0x2A6E,
    },
};

static esp_bt_uuid_t humidity_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = 0x2A6F,
    },
};

static esp_bt_uuid_t pressure_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = 0x2A6D,
    },
};

static esp_bt_uuid_t co2_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = 0x2B8C,
    },
};

static esp_bt_uuid_t voc_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = 0x2BE7,
    },
};

static esp_bt_uuid_t iaq_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {
        .uuid128 = {0xDA, 0xA7, 0xBD, 0x48, 0x12, 0x12, 0xBA, 0x82,
                    0xD6, 0x43, 0x86, 0x12, 0x98, 0x05, 0x89, 0xE2},
    },
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t temp_char_handle;
    uint16_t humid_char_handle;
    uint16_t press_char_handle;
    uint16_t co2_char_handle;
    uint16_t voc_char_handle;
    uint16_t iaq_char_handle;
    esp_bd_addr_t remote_bda;
};

static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,
    },
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

static void esp_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;

    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG_BLE, "Scanning start failed, status %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(TAG_BLE, "Scanning start success");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;

        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL,
                                                &adv_name_len);
            ESP_LOGI(TAG_BLE, "Scan result, device "ESP_BD_ADDR_STR", name len %u", ESP_BD_ADDR_HEX(scan_result->scan_rst.bda), adv_name_len);
            ESP_LOG_BUFFER_CHAR(TAG, adv_name, adv_name_len);

            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    ESP_LOGI(TAG_BLE, "Device found %s", remote_device_name);
                    if (connect == false) {
                        connect = true;
                        ESP_LOGI(TAG_BLE, "Connect to the remote device");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda,
                                           scan_result->scan_rst.ble_addr_type, true);
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
    default:
        break;
    }
}

static void esp_gattc_callback(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(TAG_BLE, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(TAG_BLE, "GATT client registered, status %d, app_id %d, gattc_if %d", param->reg.status, param->reg.app_id, gattc_if);
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret) {
            ESP_LOGE(TAG_BLE, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(TAG_BLE, "Connected, conn_id %d, remote "ESP_BD_ADDR_STR"", p_data->connect.conn_id, ESP_BD_ADDR_HEX(p_data->connect.remote_bda));
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(TAG_BLE, "Config MTU error, error code = %x", mtu_ret);
        }
        break;
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(TAG_BLE, "Open failed status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(TAG_BLE, "Open success, MTU %u", p_data->open.mtu);
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(TAG_BLE, "Service discover failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(TAG_BLE, "Service descover complete, conn_id %d", param->dis_srvc_cmpl.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(TAG_BLE, "MTU exchange, status %d, MTU %d", param->cfg_mtu.status, param->cfg_mtu.mtu);
        break;
    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(TAG_BLE, "Service search result, conn_id = %x, is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(TAG_BLE, "start handle %d, end handle %d, current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(TAG_BLE, "Service found");
            get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(TAG_BLE, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(TAG_BLE, "Service search failed, status %x", p_data->search_cmpl.status);
            break;
        }
        if(p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            ESP_LOGI(TAG_BLE, "Get service information from remote device");
        } else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGI(TAG_BLE, "Get service information from flash");
        } else {
            ESP_LOGI(TAG_BLE, "Unknown service source");
        }
        ESP_LOGI(TAG_BLE, "Service search complete, conn_id %d", p_data->search_cmpl.conn_id);

        if (get_server) {
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(
                gattc_if,
                p_data->search_cmpl.conn_id,
                ESP_GATT_DB_CHARACTERISTIC,
                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                0,
                &count
            );

            if (status != ESP_GATT_OK) {
                ESP_LOGE(TAG_BLE, "esp_ble_gattc_get_attr_count error");
                break;
            }

            if (count > 0) {
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result) {
                    ESP_LOGE(TAG_BLE, "gattc no mem");
                    break;
                }

                // Temperature
                status = esp_ble_gattc_get_char_by_uuid(
                    gattc_if,
                    p_data->search_cmpl.conn_id,
                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                    temperature_char_uuid,
                    char_elem_result,
                    &count
                );
                if (status != ESP_GATT_OK) {
                    ESP_LOGE(TAG_BLE, "esp_ble_gattc_get_char_by_uuid error");
                    free(char_elem_result);
                    char_elem_result = NULL;
                    break;
                }

                if (count > 0) {
                    ESP_LOGI(TAG_BLE, "Temperature characteristic found, handle %d", char_elem_result[0].char_handle);
                    gl_profile_tab[PROFILE_A_APP_ID].temp_char_handle = char_elem_result[0].char_handle;

                    status = esp_ble_gattc_register_for_notify(
                        gattc_if,
                        gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                        gl_profile_tab[PROFILE_A_APP_ID].temp_char_handle
                    );
                }
                else {
                    ESP_LOGE(TAG_BLE, "Temperature characteristic not found, status %d", status);
                }

                // Humidity
                status = esp_ble_gattc_get_char_by_uuid(
                    gattc_if,
                    p_data->search_cmpl.conn_id,
                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                    humidity_char_uuid,
                    char_elem_result,
                    &count
                );

                if (status != ESP_GATT_OK) {
                    ESP_LOGE(TAG_BLE, "esp_ble_gattc_get_char_by_uuid error");
                    free(char_elem_result);
                    char_elem_result = NULL;
                    break;
                }

                if (count > 0) {
                    ESP_LOGI(TAG_BLE, "Humidity characteristic found, handle %d", char_elem_result[0].char_handle);
                    gl_profile_tab[PROFILE_A_APP_ID].humid_char_handle = char_elem_result[0].char_handle;

                    status = esp_ble_gattc_register_for_notify(
                        gattc_if,
                        gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                        gl_profile_tab[PROFILE_A_APP_ID].humid_char_handle
                    );
                }
                else {
                    ESP_LOGE(TAG_BLE, "Humidity characteristic not found, status %d", status);
                }

                // Pressure
                status = esp_ble_gattc_get_char_by_uuid(
                    gattc_if,
                    p_data->search_cmpl.conn_id,
                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                    pressure_char_uuid,
                    char_elem_result,
                    &count
                );

                if (status != ESP_GATT_OK) {
                    ESP_LOGE(TAG_BLE, "esp_ble_gattc_get_char_by_uuid error");
                    free(char_elem_result);
                    char_elem_result = NULL;
                    break;
                }

                if (count > 0) {
                    ESP_LOGI(TAG_BLE, "Pressure characteristic found, handle %d", char_elem_result[0].char_handle);
                    gl_profile_tab[PROFILE_A_APP_ID].press_char_handle = char_elem_result[0].char_handle;

                    status = esp_ble_gattc_register_for_notify(
                        gattc_if,
                        gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                        gl_profile_tab[PROFILE_A_APP_ID].press_char_handle
                    );
                } else {
                    ESP_LOGE(TAG_BLE, "Pressure characteristic not found, status %d", status);
                }

                // CO2
                status = esp_ble_gattc_get_char_by_uuid(
                    gattc_if,
                    p_data->search_cmpl.conn_id,
                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                    co2_char_uuid,
                    char_elem_result,
                    &count
                );

                if (status != ESP_GATT_OK) {
                    ESP_LOGE(TAG_BLE, "esp_ble_gattc_get_char_by_uuid error");
                    free(char_elem_result);
                    char_elem_result = NULL;
                    break;
                }

                if (count > 0) {
                    ESP_LOGI(TAG_BLE, "CO2 characteristic found, handle %d", char_elem_result[0].char_handle);
                    gl_profile_tab[PROFILE_A_APP_ID].co2_char_handle = char_elem_result[0].char_handle;

                    status = esp_ble_gattc_register_for_notify(
                        gattc_if,
                        gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                        gl_profile_tab[PROFILE_A_APP_ID].co2_char_handle
                    );
                } else {
                    ESP_LOGE(TAG_BLE, "CO2 characteristic not found, status %d", status);
                }

                // VOC
                status = esp_ble_gattc_get_char_by_uuid(
                    gattc_if,
                    p_data->search_cmpl.conn_id,
                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                    voc_char_uuid,
                    char_elem_result,
                    &count
                );

                if (status != ESP_GATT_OK) {
                    ESP_LOGE(TAG_BLE, "esp_ble_gattc_get_char_by_uuid error");
                    free(char_elem_result);
                    char_elem_result = NULL;
                    break;
                }

                if (count > 0) {
                    ESP_LOGI(TAG_BLE, "VOC characteristic found, handle %d", char_elem_result[0].char_handle);
                    gl_profile_tab[PROFILE_A_APP_ID].voc_char_handle = char_elem_result[0].char_handle;

                    status = esp_ble_gattc_register_for_notify(
                        gattc_if,
                        gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                        gl_profile_tab[PROFILE_A_APP_ID].voc_char_handle
                    );
                } else {
                    ESP_LOGE(TAG_BLE, "VOC characteristic not found, status %d", status);
                }

                // IAQ
                status = esp_ble_gattc_get_char_by_uuid(
                    gattc_if,
                    p_data->search_cmpl.conn_id,
                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                    iaq_char_uuid,
                    char_elem_result,
                    &count
                );

                if (status != ESP_GATT_OK) {
                    ESP_LOGE(TAG_BLE, "esp_ble_gattc_get_char_by_uuid error");
                    free(char_elem_result);
                    char_elem_result = NULL;
                    break;
                }

                if (count > 0) {
                    ESP_LOGI(TAG_BLE, "IAQ characteristic found, handle %d", char_elem_result[0].char_handle);
                    gl_profile_tab[PROFILE_A_APP_ID].iaq_char_handle = char_elem_result[0].char_handle;

                    status = esp_ble_gattc_register_for_notify(
                        gattc_if,
                        gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                        gl_profile_tab[PROFILE_A_APP_ID].iaq_char_handle
                    );
                } else {
                    ESP_LOGE(TAG_BLE, "IAQ characteristic not found, status %d", status);
                }

                free(char_elem_result);
            }
            else {
                ESP_LOGE(TAG_BLE, "no char found");
            }
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        if (p_data->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGE(TAG_BLE, "Notification register failed status %d", p_data->reg_for_notify.status);
        }
        else {
            ESP_LOGI(TAG_BLE, "Notification register success");

            uint16_t char_handles[] = {
                gl_profile_tab[PROFILE_A_APP_ID].temp_char_handle,
                gl_profile_tab[PROFILE_A_APP_ID].humid_char_handle,
                gl_profile_tab[PROFILE_A_APP_ID].press_char_handle,
                gl_profile_tab[PROFILE_A_APP_ID].co2_char_handle,
                gl_profile_tab[PROFILE_A_APP_ID].voc_char_handle,
                gl_profile_tab[PROFILE_A_APP_ID].iaq_char_handle
            };

            for (int i = 0; i < sizeof(char_handles) / sizeof(char_handles[0]); i++) {
                uint16_t count = 0;
                uint16_t notify_en = 1;
                esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(
                    gattc_if,
                    gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                    ESP_GATT_DB_DESCRIPTOR,
                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                    char_handles[i],
                    &count
                );

                if (ret_status != ESP_GATT_OK) {
                    ESP_LOGE(TAG_BLE, "esp_ble_gattc_get_attr_count error");
                    break;
                }

                if (count > 0) {
                    descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                    if (!descr_elem_result) {
                        ESP_LOGE(TAG_BLE, "maslloc error, gattc no mem");
                        break;
                    }
                    else {
                        ret_status = esp_ble_gattc_get_descr_by_char_handle(
                            gattc_if,
                            gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                            char_handles[i],
                            notify_descr_uuid,
                            descr_elem_result,
                            &count
                        );
                        
                        if (ret_status != ESP_GATT_OK) {
                            ESP_LOGE(TAG_BLE, "esp_ble_gattc_get_descr_by_char_handle error");
                            free(descr_elem_result);
                            descr_elem_result = NULL;
                            break;
                        }

                        if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
                            ret_status = esp_ble_gattc_write_char_descr(
                                gattc_if,
                                gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                descr_elem_result[0].handle,
                                sizeof(notify_en),
                                (uint8_t *)&notify_en,
                                ESP_GATT_WRITE_TYPE_RSP,
                                ESP_GATT_AUTH_REQ_NONE
                            );
                        }

                        if (ret_status != ESP_GATT_OK) {
                            ESP_LOGE(TAG_BLE, "esp_ble_gattc_write_char_descr error for handle %d", i);
                        }
                        else {
                            ESP_LOGI(TAG_BLE, "Notifications enabled for handle %d", i);
                        }

                        free(descr_elem_result);
                    }
                }
                else {
                    ESP_LOGE(TAG_BLE, "decsr not found for handle %d", i);
                }
            }
        }
        break;
    case ESP_GATTC_READ_CHAR_EVT:
        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        break;
    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(TAG_BLE, "Notification received, handle = %d, value_len = %d", p_data->notify.handle, p_data->notify.value_len);

        if (p_data->notify.handle == gl_profile_tab[PROFILE_A_APP_ID].temp_char_handle) {
            // Temperature
            if (p_data->notify.value_len == sizeof(int16_t)) {
                int16_t raw_temperature;
                memcpy(&raw_temperature, param->notify.value, sizeof(int16_t));

                int temperature = raw_temperature / 100;

                ESP_LOGI(TAG_BLE, "Temperature: %d°C", temperature);

                if (value_temp) {
                    char temp_str[16];
                    snprintf(temp_str, sizeof(temp_str), "%d°C", temperature);
                    lv_label_set_text(value_temp, temp_str);
                }
            }
            else {
                ESP_LOGE(TAG_BLE, "Unexpected temperature value lenght: %d", p_data->notify.value_len);
            }
        }
        else if (p_data->notify.handle == gl_profile_tab[PROFILE_A_APP_ID].humid_char_handle) {
            // Humidity
            if (p_data->notify.value_len == sizeof(int16_t)) {
                int16_t raw_humidity;
                memcpy(&raw_humidity, p_data->notify.value, sizeof(int16_t));

                int humidity = raw_humidity / 100;

                ESP_LOGI(TAG_BLE, "Humidity: %d%%", humidity);

                if (value_humid) {
                    char temp_str[16];
                    snprintf(temp_str, sizeof(temp_str), "%d%%", humidity);
                    lv_label_set_text(value_humid, temp_str);
                }
            }
            else {
                ESP_LOGE(TAG_BLE, "Unexpected humidity value length: %d", p_data->notify.value_len);
            }
        }
        else if (p_data->notify.handle == gl_profile_tab[PROFILE_A_APP_ID].press_char_handle) {
            // Pressure
            if (p_data->notify.value_len == sizeof(uint32_t)) {
                uint32_t raw_pressure;
                memcpy(&raw_pressure, p_data->notify.value, sizeof(uint32_t));

                int pressure = raw_pressure / 1;

                ESP_LOGI(TAG_BLE, "Pressure: %d mmHg", pressure);

                if (value_pressure) {
                    char temp_str[16];
                    snprintf(temp_str, sizeof(temp_str), "%d mmHg", pressure);
                    lv_label_set_text(value_pressure, temp_str);
                }
            } else {
                ESP_LOGE(TAG_BLE, "Unexpected pressure value length: %d", p_data->notify.value_len);
            }
        }
        else if (p_data->notify.handle == gl_profile_tab[PROFILE_A_APP_ID].co2_char_handle) {
            // CO2
            if (p_data->notify.value_len == sizeof(int16_t)) {
                int16_t raw_co2;
                memcpy(&raw_co2, p_data->notify.value, sizeof(int16_t));

                int co2 = raw_co2 / 1;

                ESP_LOGI(TAG_BLE, "CO2: %d ppm", co2);

                if (value_co2) {
                    char temp_str[16];
                    snprintf(temp_str, sizeof(temp_str), "%d ppm", co2);
                    lv_label_set_text(value_co2, temp_str);
                }
            } else {
                ESP_LOGE(TAG_BLE, "Unexpected co2 value length: %d", p_data->notify.value_len);
            }
        }
        else if (p_data->notify.handle == gl_profile_tab[PROFILE_A_APP_ID].voc_char_handle) {
            // VOC
            if (p_data->notify.value_len == sizeof(int16_t)) {
                int16_t raw_voc;
                memcpy(&raw_voc, p_data->notify.value, sizeof(int16_t));

                int voc = raw_voc / 1;

                ESP_LOGI(TAG_BLE, "VOC: %d ppb", voc);

                if (value_voc) {
                    char temp_str[16];
                    snprintf(temp_str, sizeof(temp_str), "%d ppb", voc);
                    lv_label_set_text(value_voc, temp_str);
                }
            } else {
                ESP_LOGE(TAG_BLE, "Unexpected voc value length: %d", p_data->notify.value_len);
            }
        }
        else if (p_data->notify.handle == gl_profile_tab[PROFILE_A_APP_ID].iaq_char_handle) {
            // IAQ
            if (p_data->notify.value_len == sizeof(int16_t)) {
                int16_t raw_iaq;
                memcpy(&raw_iaq, p_data->notify.value, sizeof(int16_t));

                int iaq = raw_iaq / 1;

                ESP_LOGI(TAG_BLE, "IAQ: %d", iaq);

                if (value_iaq) {
                    char temp_str[16];
                    snprintf(temp_str, sizeof(temp_str), "%d", iaq);
                    lv_label_set_text(value_iaq, temp_str);
                }
            } else {
                ESP_LOGE(TAG_BLE, "Unexpected iaq value length: %d", p_data->notify.value_len);
            }
        }
        else {
            ESP_LOGE(TAG_BLE, "Unknown handle: %d", p_data->notify.handle);
        }
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(TAG_BLE, "Disconnected from device, remote "ESP_BD_ADDR_STR", reason 0x%02x", ESP_BD_ADDR_HEX(p_data->disconnect.remote_bda), p_data->disconnect.reason);
        connect = false;
        get_server = false;
        break;
    default:
        break;
    }
}

//

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

    // BLE
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(esp_gap_callback);
    if (ret){
        ESP_LOGE(TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_register_callback(esp_gattc_callback);
    if(ret){
        ESP_LOGE(TAG, "%s gattc register failed, error code = %x", __func__, ret);
        return;
    }

    esp_ble_gattc_app_register(PROFILE_A_APP_ID);

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_timer_handler();
    }
}