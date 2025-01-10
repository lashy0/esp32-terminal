#include <stdlib.h>
#include <sys/cdefs.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "lcd_panel_ili9488";

// ILI9488 Register Definitions

#define ILI9488_COLOR_MODE_16BIT                    0x55
#define ILI9488_COLOR_MODE_18BIT                    0x66

/* Level 1 Commands (from the display Datasheet) */
#define ILI9488_CMD_NOP                             0x00
#define ILI9488_CMD_SOFTWARE_RESET                  0x01
#define ILI9488_CMD_READ_DISP_ID                    0x04
#define ILI9488_CMD_READ_ERROR_DSI                  0x05
#define ILI9488_CMD_READ_DISP_STATUS                0x09
#define ILI9488_CMD_READ_DISP_POWER_MODE            0x0A
#define ILI9488_CMD_READ_DISP_MADCTRL               0x0B
#define ILI9488_CMD_READ_DISP_PIXEL_FORMAT          0x0C
#define ILI9488_CMD_READ_DISP_IMAGE_MODE            0x0D
#define ILI9488_CMD_READ_DISP_SIGNAL_MODE           0x0E
#define ILI9488_CMD_READ_DISP_SELF_DIAGNOSTIC       0x0F
#define ILI9488_CMD_ENTER_SLEEP_MODE                0x10
#define ILI9488_CMD_SLEEP_OUT                       0x11
#define ILI9488_CMD_PARTIAL_MODE_ON                 0x12
#define ILI9488_CMD_NORMAL_DISP_MODE_ON             0x13
#define ILI9488_CMD_DISP_INVERSION_OFF              0x20
#define ILI9488_CMD_DISP_INVERSION_ON               0x21
#define ILI9488_CMD_PIXEL_OFF                       0x22
#define ILI9488_CMD_PIXEL_ON                        0x23
#define ILI9488_CMD_DISPLAY_OFF                     0x28
#define ILI9488_CMD_DISPLAY_ON                      0x29
#define ILI9488_CMD_COLUMN_ADDRESS_SET              0x2A
#define ILI9488_CMD_PAGE_ADDRESS_SET                0x2B
#define ILI9488_CMD_MEMORY_WRITE                    0x2C
#define ILI9488_CMD_MEMORY_READ                     0x2E
#define ILI9488_CMD_PARTIAL_AREA                    0x30
#define ILI9488_CMD_VERT_SCROLL_DEFINITION          0x33
#define ILI9488_CMD_TEARING_EFFECT_LINE_OFF         0x34
#define ILI9488_CMD_TEARING_EFFECT_LINE_ON          0x35
#define ILI9488_CMD_MEMORY_ACCESS_CONTROL           0x36
#define ILI9488_CMD_VERT_SCROLL_START_ADDRESS       0x37
#define ILI9488_CMD_IDLE_MODE_OFF                   0x38
#define ILI9488_CMD_IDLE_MODE_ON                    0x39
#define ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET         0x3A
#define ILI9488_CMD_WRITE_MEMORY_CONTINUE           0x3C
#define ILI9488_CMD_READ_MEMORY_CONTINUE            0x3E
#define ILI9488_CMD_SET_TEAR_SCANLINE               0x44
#define ILI9488_CMD_GET_SCANLINE                    0x45
#define ILI9488_CMD_WRITE_DISPLAY_BRIGHTNESS        0x51
#define ILI9488_CMD_READ_DISPLAY_BRIGHTNESS         0x52
#define ILI9488_CMD_WRITE_CTRL_DISPLAY              0x53
#define ILI9488_CMD_READ_CTRL_DISPLAY               0x54
#define ILI9488_CMD_WRITE_CONTENT_ADAPT_BRIGHTNESS  0x55
#define ILI9488_CMD_READ_CONTENT_ADAPT_BRIGHTNESS   0x56
#define ILI9488_CMD_WRITE_MIN_CAB_LEVEL             0x5E
#define ILI9488_CMD_READ_MIN_CAB_LEVEL              0x5F
#define ILI9488_CMD_READ_ABC_SELF_DIAG_RES          0x68
#define ILI9488_CMD_READ_ID1                        0xDA
#define ILI9488_CMD_READ_ID2                        0xDB
#define ILI9488_CMD_READ_ID3                        0xDC

/* Level 2 Commands (from the display Datasheet) */
#define ILI9488_CMD_INTERFACE_MODE_CONTROL          0xB0
#define ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL       0xB1
#define ILI9488_CMD_FRAME_RATE_CONTROL_IDLE_8COLOR  0xB2
#define ILI9488_CMD_FRAME_RATE_CONTROL_PARTIAL      0xB3
#define ILI9488_CMD_DISPLAY_INVERSION_CONTROL       0xB4
#define ILI9488_CMD_BLANKING_PORCH_CONTROL          0xB5
#define ILI9488_CMD_DISPLAY_FUNCTION_CONTROL        0xB6
#define ILI9488_CMD_ENTRY_MODE_SET                  0xB7
#define ILI9488_CMD_BACKLIGHT_CONTROL_1             0xB9
#define ILI9488_CMD_BACKLIGHT_CONTROL_2             0xBA
#define ILI9488_CMD_HS_LANES_CONTROL                0xBE
#define ILI9488_CMD_POWER_CONTROL_1                 0xC0
#define ILI9488_CMD_POWER_CONTROL_2                 0xC1
#define ILI9488_CMD_POWER_CONTROL_NORMAL_3          0xC2
#define ILI9488_CMD_POWER_CONTROL_IDEL_4            0xC3
#define ILI9488_CMD_POWER_CONTROL_PARTIAL_5         0xC4
#define ILI9488_CMD_VCOM_CONTROL_1                  0xC5
#define ILI9488_CMD_CABC_CONTROL_1                  0xC6
#define ILI9488_CMD_CABC_CONTROL_2                  0xC8
#define ILI9488_CMD_CABC_CONTROL_3                  0xC9
#define ILI9488_CMD_CABC_CONTROL_4                  0xCA
#define ILI9488_CMD_CABC_CONTROL_5                  0xCB
#define ILI9488_CMD_CABC_CONTROL_6                  0xCC
#define ILI9488_CMD_CABC_CONTROL_7                  0xCD
#define ILI9488_CMD_CABC_CONTROL_8                  0xCE
#define ILI9488_CMD_CABC_CONTROL_9                  0xCF
#define ILI9488_CMD_NVMEM_WRITE                     0xD0
#define ILI9488_CMD_NVMEM_PROTECTION_KEY            0xD1
#define ILI9488_CMD_NVMEM_STATUS_READ               0xD2
#define ILI9488_CMD_READ_ID4                        0xD3
#define ILI9488_CMD_ADJUST_CONTROL_1                0xD7
#define ILI9488_CMD_READ_ID_VERSION                 0xD8
#define ILI9488_CMD_POSITIVE_GAMMA_CORRECTION       0xE0
#define ILI9488_CMD_NEGATIVE_GAMMA_CORRECTION       0xE1
#define ILI9488_CMD_DIGITAL_GAMMA_CONTROL_1         0xE2
#define ILI9488_CMD_DIGITAL_GAMMA_CONTROL_2         0xE3
#define ILI9488_CMD_SET_IMAGE_FUNCTION              0xE9
#define ILI9488_CMD_ADJUST_CONTROL_2                0xF2
#define ILI9488_CMD_ADJUST_CONTROL_3                0xF7
#define ILI9488_CMD_ADJUST_CONTROL_4                0xF8
#define ILI9488_CMD_ADJUST_CONTROL_5                0xF9
#define ILI9488_CMD_SPI_READ_SETTINGS               0xFB
#define ILI9488_CMD_ADJUST_CONTROL_6                0xFC
#define ILI9488_CMD_ADJUST_CONTROL_7                0xFF

static esp_err_t panel_ili9488_del(esp_lcd_panel_t *panel);
static esp_err_t panel_ili9488_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_ili9488_init(esp_lcd_panel_t *panel);
static esp_err_t panel_ili9488_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end,
                                           const void *color_data);
static esp_err_t panel_ili9488_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_ili9488_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_ili9488_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_ili9488_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_ili9488_disp_on_off(esp_lcd_panel_t *panel, bool on_off);
static esp_err_t panel_ili9488_sleep(esp_lcd_panel_t *panel, bool sleep);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val;
    uint8_t colmod_val;
    uint8_t *color_buffer;
} ili9488_panel_t;

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

esp_err_t esp_lcd_new_panel_ili9488(const esp_lcd_panel_io_handle_t io,
                                    const esp_lcd_panel_dev_config_t *panel_dev_config,
                                    const size_t buffer_size,
                                    esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    ili9488_panel_t *ili9488 = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    ili9488 = (ili9488_panel_t *)calloc(1, sizeof(ili9488_panel_t));
    ESP_GOTO_ON_FALSE(ili9488, ESP_ERR_NO_MEM, err, TAG, "no mem for ili9488 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    ili9488->madctl_val = LCD_CMD_MX_BIT | LCD_CMD_BGR_BIT;
    switch (panel_dev_config->rgb_ele_order) {
    case LCD_RGB_ELEMENT_ORDER_RGB:
        ili9488->madctl_val &= ~LCD_CMD_BGR_BIT;
        break;
    case LCD_RGB_ELEMENT_ORDER_BGR:
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "Unsupported color space");
        break;
    }

    uint8_t fb_bits_per_pixel = 0;
    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        ili9488->colmod_val = ILI9488_COLOR_MODE_16BIT;
        fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666
        ili9488->colmod_val = ILI9488_COLOR_MODE_18BIT;
        fb_bits_per_pixel = 18;
        ili9488->color_buffer = (uint8_t *)heap_caps_malloc(buffer_size * 3, MALLOC_CAP_DMA);
        ESP_GOTO_ON_FALSE(ili9488->color_buffer, ESP_ERR_NO_MEM, err, TAG, "no mem for ili9488 color buffer");
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "Unsupported bits per pixel");
        break;
    }

    ili9488->io = io;
    ili9488->fb_bits_per_pixel = fb_bits_per_pixel;
    ili9488->reset_gpio_num = panel_dev_config->reset_gpio_num;
    ili9488->reset_level = panel_dev_config->flags.reset_active_high;
    ili9488->base.del = panel_ili9488_del;
    ili9488->base.reset = panel_ili9488_reset;
    ili9488->base.init = panel_ili9488_init;
    ili9488->base.draw_bitmap = panel_ili9488_draw_bitmap;
    ili9488->base.invert_color = panel_ili9488_invert_color;
    ili9488->base.mirror = panel_ili9488_mirror;
    ili9488->base.swap_xy = panel_ili9488_swap_xy;
    ili9488->base.set_gap = panel_ili9488_set_gap;
    ili9488->base.disp_on_off = panel_ili9488_disp_on_off;
    ili9488->base.disp_sleep = panel_ili9488_sleep;

    *ret_panel = &(ili9488->base);

    ESP_LOGI(TAG, "new ili9488 panel @%p", ili9488);

    return ESP_OK;

err:
    if (ili9488) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        if (ili9488->color_buffer != NULL) {
            heap_caps_free(ili9488->color_buffer);
        }
        free(ili9488);
    }
    return ret;
}

static esp_err_t panel_ili9488_del(esp_lcd_panel_t *panel)
{
    ili9488_panel_t *ili9488 = __containerof(panel, ili9488_panel_t, base);

    if (ili9488->reset_gpio_num >= 0) {
        gpio_reset_pin(ili9488->reset_gpio_num);
    }
    if (ili9488->color_buffer != NULL) {
        heap_caps_free(ili9488->color_buffer);
    }
    ESP_LOGD(TAG, "del ili9488 panel @%p", ili9488);
    free(ili9488);
    return ESP_OK;
}

static esp_err_t panel_ili9488_reset(esp_lcd_panel_t *panel)
{
    ili9488_panel_t *ili9488 = __containerof(panel, ili9488_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9488->io;

    if (ili9488->reset_gpio_num >= 0) {
        gpio_set_level(ili9488->reset_gpio_num, ili9488->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(ili9488->reset_gpio_num, !ili9488->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    else {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG,
                            "send command failed");
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return ESP_OK;
}

static esp_err_t panel_ili9488_init(esp_lcd_panel_t *panel)
{
    ili9488_panel_t *ili9488 = __containerof(panel, ili9488_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9488->io;

    lcd_init_cmd_t ili9488_init_cmds[] = {
        {ILI9488_CMD_SLEEP_OUT, {0x00}, 0x80},
        {ILI9488_CMD_POSITIVE_GAMMA_CORRECTION,
            {0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F}, 15
        },
        {ILI9488_CMD_NEGATIVE_GAMMA_CORRECTION,
            {0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F}, 15
        },
        {ILI9488_CMD_POWER_CONTROL_1, {0x17, 0x15}, 2},
        {ILI9488_CMD_POWER_CONTROL_2, {0x41}, 1},
        {ILI9488_CMD_VCOM_CONTROL_1, {0x00, 0x12, 0x80}, 3},
        {ILI9488_CMD_MEMORY_ACCESS_CONTROL, {(ili9488->madctl_val)}, 1}, // ? что сюда
        {ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET, {ili9488->colmod_val}, 1},
        {ILI9488_CMD_INTERFACE_MODE_CONTROL, {0x00}, 1},
        {ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL, {0xA0}, 1},
        {ILI9488_CMD_DISPLAY_INVERSION_CONTROL, {0x02}, 1},
        {ILI9488_CMD_DISPLAY_FUNCTION_CONTROL, {0x02, 0x02}, 2},
        {ILI9488_CMD_SET_IMAGE_FUNCTION, {0x00}, 1},
        {ILI9488_CMD_WRITE_CTRL_DISPLAY, {0x28}, 1},
        {ILI9488_CMD_WRITE_DISPLAY_BRIGHTNESS, {0x7F}, 1},
        {ILI9488_CMD_ADJUST_CONTROL_3, {0xA9, 0x51, 0x2C, 0x02}, 4},
        {ILI9488_CMD_DISPLAY_ON, {0x00}, 0x80},
        {0, {0}, 0xff},
    };

    int cmd = 0;
    while (ili9488_init_cmds[cmd].databytes != 0xff) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, ili9488_init_cmds[cmd].cmd,
                                                      ili9488_init_cmds[cmd].data,
                                                      ili9488_init_cmds[cmd].databytes & 0x1F), TAG,
                            "send command failed");
        cmd++;
    }

    // Заменить на ILI9488_CMD_DISPLAY_ON и ILI9488_CMD_SLEEP_OUT
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0), TAG,
                        "io tx param failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_DISPON, NULL, 0), TAG,
                        "io tx param failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

static esp_err_t panel_ili9488_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end,
                                           const void *color_data)
{
    ili9488_panel_t *ili9488 = __containerof(panel, ili9488_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9488->io;

    x_start += ili9488->x_gap;
    x_end += ili9488->x_gap;
    y_start += ili9488->y_gap;
    y_end += ili9488->y_gap;

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]) {
        (x_start >> 8) & 0xFF,
        x_start & 0xFF,
        ((x_end - 1) >> 8) & 0xFF,
        (x_end - 1) & 0xFF,
    }, 4), TAG, "io tx param failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]) {
        (y_start >> 8) & 0xFF,
        y_start & 0xFF,
        ((y_end - 1) >> 8) & 0xFF,
        (y_end - 1) & 0xFF,
    }, 4), TAG, "io tx param failed");

    size_t len = (x_end - x_start) * (y_end - y_start);

    if (ili9488->fb_bits_per_pixel == 18) {
        uint8_t *buf = ili9488->color_buffer;
        uint16_t *raw_color_data = (uint16_t *)color_data;

        for (uint32_t i = 0, pixel_index = 0; i < len; i++) {
            buf[pixel_index++] = (uint8_t) (((raw_color_data[i] & 0xF800) >> 8) |
                                            ((raw_color_data[i] & 0x8000) >> 13));
            buf[pixel_index++] = (uint8_t) ((raw_color_data[i] & 0x07E0) >> 3);
            buf[pixel_index++] = (uint8_t) (((raw_color_data[i] & 0x001F) << 3) |
                                            ((raw_color_data[i] & 0x0010) >> 2));
        }

        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, buf, len * 3), TAG,
                        "io tx color failed");
    }
    else {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len * 2), TAG,
                        "io tx color failed");
    }

    
    return ESP_OK;
}

static esp_err_t panel_ili9488_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    ili9488_panel_t *ili9488 = __containerof(panel, ili9488_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9488->io;
    int command = 0;

    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "io tx param failed");

    return ESP_OK;
}

static esp_err_t panel_ili9488_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    ili9488_panel_t *ili9488 = __containerof(panel, ili9488_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9488->io;

    if (mirror_x) {
        ili9488->madctl_val &= ~LCD_CMD_MX_BIT;
    } else {
        ili9488->madctl_val |= LCD_CMD_MX_BIT;
    }
    if (mirror_y) {
        ili9488->madctl_val |= LCD_CMD_MY_BIT;
    } else {
        ili9488->madctl_val &= ~LCD_CMD_MY_BIT;
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        ili9488->madctl_val
    }, 1), TAG, "io tx param failed");

    return ESP_OK;
}

static esp_err_t panel_ili9488_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    ili9488_panel_t *ili9488 = __containerof(panel, ili9488_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9488->io;

    if (swap_axes) {
        ili9488->madctl_val |= LCD_CMD_MV_BIT;
    } else {
        ili9488->madctl_val &= ~LCD_CMD_MV_BIT;
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        ili9488->madctl_val
    }, 1), TAG, "io tx param failed");

    return ESP_OK;
}

static esp_err_t panel_ili9488_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    ili9488_panel_t *ili9488 = __containerof(panel, ili9488_panel_t, base);
    
    ili9488->x_gap = x_gap;
    ili9488->y_gap = y_gap;

    return ESP_OK;
}

static esp_err_t panel_ili9488_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    ili9488_panel_t *ili9488 = __containerof(panel, ili9488_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9488->io;
    int command = 0;

    if (on_off) {
        command = LCD_CMD_DISPON; // ILI9488_CMD_DISPLAY_ON
    } else {
        command = LCD_CMD_DISPOFF; // ILI9488_CMD_DISPLAY_OFF
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    return ESP_OK;
}

static esp_err_t panel_ili9488_sleep(esp_lcd_panel_t *panel, bool sleep)
{
    ili9488_panel_t *ili9488 = __containerof(panel, ili9488_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9488->io;
    int command = 0;

    if (sleep) {
        command = ILI9488_CMD_ENTER_SLEEP_MODE;
    } else {
        command = ILI9488_CMD_SLEEP_OUT;
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}