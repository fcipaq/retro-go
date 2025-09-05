/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc/soc_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_lcd_hd395003c30.h"

#define HD395003C30_CMD_SHLR_BIT    (1ULL << 0)
#define HD395003C30_CMD_UPDN_BIT    (1ULL << 1)

typedef struct {
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    const hd395003c30_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
    struct {
        unsigned int reset_level: 1;
    } flags;
    // To save the original functions of MIPI DPI panel
    esp_err_t (*del)(esp_lcd_panel_t *panel);
    esp_err_t (*init)(esp_lcd_panel_t *panel);
} hd395003c30_panel_t;

static const char *TAG = "hd395003c30";

static esp_err_t panel_hd395003c30_send_init_cmds(hd395003c30_panel_t *hd395003c30);

static esp_err_t panel_hd395003c30_del(esp_lcd_panel_t *panel);
static esp_err_t panel_hd395003c30_init(esp_lcd_panel_t *panel);
static esp_err_t panel_hd395003c30_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_hd395003c30_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_hd395003c30_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_hd395003c30_disp_on_off(esp_lcd_panel_t *panel, bool on_off);

esp_err_t esp_lcd_new_panel_hd395003c30(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
                                    esp_lcd_panel_handle_t *ret_panel, int lcd_scale)
{
/*
    ESP_LOGI(TAG, "version: %d.%d.%d", ESP_LCD_HD395003C30_VER_MAJOR, ESP_LCD_HD395003C30_VER_MINOR,
             ESP_LCD_HD395003C30_VER_PATCH);
*/
    ESP_RETURN_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, TAG, "invalid arguments");
    hd395003c30_vendor_config_t *vendor_config = (hd395003c30_vendor_config_t *)panel_dev_config->vendor_config;
    ESP_RETURN_ON_FALSE(vendor_config && vendor_config->mipi_config.dpi_config && vendor_config->mipi_config.dsi_bus, ESP_ERR_INVALID_ARG, TAG,
                        "invalid vendor config");

    esp_err_t ret = ESP_OK;
    hd395003c30_panel_t *hd395003c30 = (hd395003c30_panel_t *)calloc(1, sizeof(hd395003c30_panel_t));
    ESP_RETURN_ON_FALSE(hd395003c30, ESP_ERR_NO_MEM, TAG, "no mem for hd395003c30 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    hd395003c30->io = io;
    hd395003c30->init_cmds = vendor_config->init_cmds;
    hd395003c30->init_cmds_size = vendor_config->init_cmds_size;
    hd395003c30->reset_gpio_num = panel_dev_config->reset_gpio_num;
    hd395003c30->flags.reset_level = panel_dev_config->flags.reset_active_high;

    // Create MIPI DPI panel
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_dpi(vendor_config->mipi_config.dsi_bus, vendor_config->mipi_config.dpi_config, ret_panel, lcd_scale), err, TAG,
                      "create MIPI DPI panel failed");
    ESP_LOGD(TAG, "new MIPI DPI panel @%p", *ret_panel);

    // Save the original functions of MIPI DPI panel
    hd395003c30->del = (*ret_panel)->del;
    hd395003c30->init = (*ret_panel)->init;
    // Overwrite the functions of MIPI DPI panel
    (*ret_panel)->del = panel_hd395003c30_del;
    (*ret_panel)->init = panel_hd395003c30_init;
    (*ret_panel)->reset = panel_hd395003c30_reset;
    (*ret_panel)->mirror = panel_hd395003c30_mirror;
    (*ret_panel)->invert_color = panel_hd395003c30_invert_color;
    (*ret_panel)->disp_on_off = panel_hd395003c30_disp_on_off;
    (*ret_panel)->user_data = hd395003c30;
    ESP_LOGD(TAG, "new hd395003c30 panel @%p", hd395003c30);

    return ESP_OK;

err:
    if (hd395003c30) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(hd395003c30);
    }
    return ret;
}

static const hd395003c30_lcd_init_cmd_t vendor_specific_init_default[] = {
//  {cmd, { data }, data_size, delay_ms}
  {0xB9, (uint8_t []) {0xF1, 0x12, 0x87}, 3, 0},

  {0xB2, (uint8_t []) {0xB4, 0x03, 0X70}, 3, 0},

  {0xB3, (uint8_t []) {0x10, 0x10, 0x28, 0x28, 0x03, 0xFF, 0x00, 0x00, 0x00, 0x00}, 10, 0},

  {0xB4, (uint8_t []) {0x80}, 1, 0},

  {0xB5, (uint8_t []) {0x0A, 0x0A}, 2, 0},

  {0xB6, (uint8_t []) {0x8D, 0x8D}, 2, 0},

  {0xB8, (uint8_t []) {0x26, 0x22, 0xF0, 0x13}, 4, 0},

  {0xBA, (uint8_t []) {0x31, 0x81, 0x05, 0xF9, 0x0E, 0x0E, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x25, 0x00, 0x91, 0x0A, 0x00, 0x00, 0x01, 0x4F, 0x01, 0x00, 0x00, 0x37}, 27, 0},

  {0xBC, (uint8_t []) {0x47}, 1, 0},

  {0xBF, (uint8_t []) {0x02, 0x10, 0x00, 0x80, 0x04}, 5, 0},

  {0xC0, (uint8_t []) {0x73, 0x73, 0x50, 0x50, 0x00, 0x00, 0x12, 0x73, 0x00}, 9, 0},

  {0xC1, (uint8_t []) {0x36, 0x00, 0x32, 0x32, 0x77, 0xE1, 0x77, 0x77, 0xCC, 0xCC, 0xFF, 0xFF, 0x11, 0x11, 0x00, 0x00, 0x32}, 17, 0},

  {0xC7, (uint8_t []) {0x10, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0xED, 0xC5, 0x00, 0xA5}, 12, 0},

  {0xC8, (uint8_t []) {0x10, 0x40, 0x1E, 0x03}, 4, 0},

  {0xCC, (uint8_t []) {0x0B}, 1, 0},

  {0xE0, (uint8_t []) {0x00, 0x0A, 0x0F, 0x2A, 0x33, 0x3F, 0x44, 0x39, 0x06, 0x0C, 0x0E, 0x14, 0x15, 0x13, 0x15, 0x10, 0x18, 0x00, 0x0A, 0x0F, 0x2A, 0x33, 0x3F, 0x44, 0x39, 0x06, 0x0C, 0x0E, 0x14, 0x15, 0x13, 0x15, 0x10, 0x18}, 34, 0},

  {0xE1, (uint8_t []) {0x11, 0x11, 0x91, 0x00, 0x00, 0x00, 0x00}, 7, 0},

  {0xE3, (uint8_t []) {0x07, 0x07, 0x0B, 0x0B, 0x0B, 0x0B, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x04, 0xC0, 0x10}, 14, 0},

  {0xE9, (uint8_t []) {0xC8, 0x10, 0x0A, 0x00, 0x00, 0x80, 0x81, 0x12, 0x31, 0x23, 0x4F, 0x86, 0xA0, 0x00, 0x47, 0x08, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x98, 0x02, 0x8B, 0xAF, 0x46, 0x02, 0x88, 0x88, 0x88, 0x88, 0x88, 0x98, 0x13, 0x8B, 0xAF, 0x57, 0x13, 0x88, 0x88, 0x88, 0x88, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 63, 0},

  {0xEA, (uint8_t []) {0x97, 0x0C, 0x09, 0x09, 0x09, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9F, 0x31, 0x8B, 0xA8, 0x31, 0x75, 0x88, 0x88, 0x88, 0x88, 0x88, 0x9F, 0x20, 0x8B, 0xA8, 0x20, 0x64, 0x88, 0x88, 0x88, 0x88, 0x88, 0x23, 0x00, 0x00, 0x02, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x80, 0x81, 0x00, 0x00, 0x00, 0x00}, 61, 0},

  {0xEF, (uint8_t []) {0xFF, 0xFF, 0x01}, 3, 0},

  // TE on (vsync mode)
  //{0xC7, (uint8_t []) {0x10, 0x00}, 2, 0},

  {0x11, (uint8_t []) {0x00}, 0, 250},

  {0x29, (uint8_t []) {0x00}, 0, 50},


};

static esp_err_t panel_hd395003c30_send_init_cmds(hd395003c30_panel_t *hd395003c30)
{
    esp_lcd_panel_io_handle_t io = hd395003c30->io;
    const hd395003c30_lcd_init_cmd_t *init_cmds = NULL;
    uint16_t init_cmds_size = 0;

    // vendor specific initialization, it can be different between manufacturers
    // should consult the LCD supplier for initialization sequence code
    if (hd395003c30->init_cmds) {
        init_cmds = hd395003c30->init_cmds;
        init_cmds_size = hd395003c30->init_cmds_size;
    } else {
        init_cmds = vendor_specific_init_default;
        init_cmds_size = sizeof(vendor_specific_init_default) / sizeof(hd395003c30_lcd_init_cmd_t);
    }

    for (int i = 0; i < init_cmds_size; i++) {
        // Send command
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, init_cmds[i].cmd, init_cmds[i].data, init_cmds[i].data_bytes),
                            TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(init_cmds[i].delay_ms));
    }
    ESP_LOGD(TAG, "send init commands success");

    return ESP_OK;
}

static esp_err_t panel_hd395003c30_del(esp_lcd_panel_t *panel)
{
    hd395003c30_panel_t *hd395003c30 = (hd395003c30_panel_t *)panel->user_data;

    if (hd395003c30->reset_gpio_num >= 0) {
        gpio_reset_pin(hd395003c30->reset_gpio_num);
    }
    // Delete MIPI DPI panel
    hd395003c30->del(panel);
    free(hd395003c30);
    ESP_LOGD(TAG, "del hd395003c30 panel @%p", hd395003c30);

    return ESP_OK;
}

static esp_err_t panel_hd395003c30_init(esp_lcd_panel_t *panel)
{
    hd395003c30_panel_t *hd395003c30 = (hd395003c30_panel_t *)panel->user_data;

    ESP_RETURN_ON_ERROR(panel_hd395003c30_send_init_cmds(hd395003c30), TAG, "send init commands failed");
    ESP_RETURN_ON_ERROR(hd395003c30->init(panel), TAG, "init MIPI DPI panel failed");

    return ESP_OK;
}

static esp_err_t panel_hd395003c30_reset(esp_lcd_panel_t *panel)
{
    hd395003c30_panel_t *hd395003c30 = (hd395003c30_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io = hd395003c30->io;

    // Perform hardware reset
    if (hd395003c30->reset_gpio_num >= 0) {
        gpio_set_level(hd395003c30->reset_gpio_num, hd395003c30->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(hd395003c30->reset_gpio_num, !hd395003c30->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(20));
    } else if (io) { // Perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return ESP_OK;
}

static esp_err_t panel_hd395003c30_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    hd395003c30_panel_t *hd395003c30 = (hd395003c30_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io = hd395003c30->io;
    uint8_t madctl_val = 0x01;

    ESP_RETURN_ON_FALSE(io, ESP_ERR_INVALID_STATE, TAG, "invalid panel IO");

    // Control mirror through LCD command
    if (mirror_x) {
        madctl_val |= HD395003C30_CMD_SHLR_BIT;
    } else {
        madctl_val &= ~HD395003C30_CMD_SHLR_BIT;
    }
    if (mirror_y) {
        madctl_val |= HD395003C30_CMD_UPDN_BIT;
    } else {
        madctl_val &= ~HD395003C30_CMD_UPDN_BIT;
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t []) {
        madctl_val
    }, 1), TAG, "send command failed");

    return ESP_OK;
}

static esp_err_t panel_hd395003c30_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    hd395003c30_panel_t *hd395003c30 = (hd395003c30_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io = hd395003c30->io;
    uint8_t command = 0;

    ESP_RETURN_ON_FALSE(io, ESP_ERR_INVALID_STATE, TAG, "invalid panel IO");

    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");

    return ESP_OK;
}

static esp_err_t panel_hd395003c30_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    ESP_LOGE(TAG, "display on/off is not supported");

    return ESP_ERR_NOT_SUPPORTED;
}
