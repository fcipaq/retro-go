/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include "soc/soc_caps.h"

#if SOC_MIPI_DSI_SUPPORTED

#include "soc/soc_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"

#define EK79007_CMD_SHLR_BIT    (1ULL << 0)
#define EK79007_CMD_UPDN_BIT    (1ULL << 1)

#define LCD_H_RES (1024)
#define LCD_V_RES (600)
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV (2500)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LCD panel initialization commands.
 *
 */
typedef struct {
    int cmd;                /*<! The specific LCD command */
    const void *data;       /*<! Buffer that holds the command specific data */
    size_t data_bytes;      /*<! Size of `data` in memory, in bytes */
    unsigned int delay_ms;  /*<! Delay in milliseconds after this command */
} ek79007_lcd_init_cmd_t;

/**
 * @brief LCD panel vendor configuration.
 *
 * @note  This structure needs to be passed to the `vendor_config` field in `esp_lcd_panel_dev_config_t`.
 *
 */
typedef struct {
    const ek79007_lcd_init_cmd_t *init_cmds;         /*!< Pointer to initialization commands array. Set to NULL if using default commands.
                                                     *   The array should be declared as `static const` and positioned outside the function.
                                                     *   Please refer to `vendor_specific_init_default` in source file.
                                                     */
    uint16_t init_cmds_size;                        /*<! Number of commands in above array */
    struct {
        esp_lcd_dsi_bus_handle_t dsi_bus;               /*!< MIPI-DSI bus configuration */
        const esp_lcd_dpi_panel_config_t *dpi_config;   /*!< MIPI-DPI panel configuration */
    } mipi_config;
    struct {
        unsigned int use_mipi_interface: 1;         /*<! Set to 1 if using MIPI interface, default is RGB interface */
        unsigned int mirror_by_cmd: 1;              /*<! The `mirror()` function will be implemented by LCD command if set to 1. This flag is only valid for the RGB interface.
                                                     *   Otherwise, the function will be implemented by software.
                                                     */

        union {
            unsigned int auto_del_panel_io: 1;
            unsigned int enable_io_multiplex: 1;
        };  /*<! Delete the panel IO instance automatically if set to 1. All `*_by_cmd` flags will be invalid.
             *   If the panel IO pins are sharing other pins of the RGB interface to save GPIOs,
             *   Please set it to 1 to release the panel IO and its pins (except CS signal).
             *   This flag is only valid for the RGB interface.
             */
    } flags;
} panel_vendor_config_t;

/**
 * @brief MIPI DSI bus configuration structure
 *
 */
#define PANEL_BUS_DSI_2CH_CONFIG()                \
    {                                                     \
        .bus_id = 0,                                      \
        .num_data_lanes = 2,                              \
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,      \
        .lane_bit_rate_mbps = 1000,                       \
    }

/**
 * @brief MIPI DBI panel IO configuration structure
 *
 */
#define PANEL_IO_DBI_CONFIG() \
    {                                 \
        .virtual_channel = 0,         \
        .lcd_cmd_bits = 8,            \
        .lcd_param_bits = 8,          \
    }

/**
 * @brief MIPI DPI configuration structure
 *
 * @note  refresh_rate = (dpi_clock_freq_mhz * 1000000) / (h_res + hsync_pulse_width + hsync_back_porch + hsync_front_porch)
 *                                                      / (v_res + vsync_pulse_width + vsync_back_porch + vsync_front_porch)
 *
 */
#define PANEL_60HZ_CONFIG(px_format)            \
    {                                                            \
        .virtual_channel = 0,                                    \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,             \
        .dpi_clock_freq_mhz = 51,                                \
        .pixel_format = px_format,                               \
        .num_fbs = 3,                                            \
        .video_timing = {                                        \
            .h_size = 1024,                                      \
            .v_size = 600,                                       \
            .hsync_pulse_width = 10,                             \
            .hsync_back_porch = 160,                             \
            .hsync_front_porch = 160,                            \
            .vsync_pulse_width = 1,                              \
            .vsync_back_porch = 23,                              \
            .vsync_front_porch = 12,                             \
        },                                                       \
        .flags = {                                               \
          .use_dma2d = true,                                     \
        },                                                       \
    }

#ifdef __cplusplus
}
#endif

typedef struct {
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    const ek79007_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
    struct {
        unsigned int reset_level: 1;
    } flags;
    // To save the original functions of MIPI DPI panel
    esp_err_t (*del)(esp_lcd_panel_t *panel);
    esp_err_t (*init)(esp_lcd_panel_t *panel);
} ek79007_panel_t;

static const char *PANEL_TAG = "ek79007";

static esp_err_t panel_ek79007_send_init_cmds(ek79007_panel_t *ek79007);

static esp_err_t panel_ek79007_del(esp_lcd_panel_t *panel);
static esp_err_t panel_ek79007_init(esp_lcd_panel_t *panel);
static esp_err_t panel_ek79007_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_ek79007_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_ek79007_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_ek79007_disp_on_off(esp_lcd_panel_t *panel, bool on_off);

esp_err_t esp_lcd_new_panel(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
                                    esp_lcd_panel_handle_t *ret_panel)
{
/*
    ESP_LOGI(PANEL_TAG, "version: %d.%d.%d", ESP_LCD_EK79007_VER_MAJOR, ESP_LCD_EK79007_VER_MINOR,
             ESP_LCD_EK79007_VER_PATCH);
*/
    ESP_RETURN_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, PANEL_TAG, "invalid arguments");
    panel_vendor_config_t *vendor_config = (panel_vendor_config_t *)panel_dev_config->vendor_config;
    ESP_RETURN_ON_FALSE(vendor_config && vendor_config->mipi_config.dpi_config && vendor_config->mipi_config.dsi_bus, ESP_ERR_INVALID_ARG, PANEL_TAG,
                        "invalid vendor config");

    esp_err_t ret = ESP_OK;
    ek79007_panel_t *ek79007 = (ek79007_panel_t *)calloc(1, sizeof(ek79007_panel_t));
    ESP_RETURN_ON_FALSE(ek79007, ESP_ERR_NO_MEM, PANEL_TAG, "no mem for ek79007 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, PANEL_TAG, "configure GPIO for RST line failed");
    }

    ek79007->io = io;
    ek79007->init_cmds = vendor_config->init_cmds;
    ek79007->init_cmds_size = vendor_config->init_cmds_size;
    ek79007->reset_gpio_num = panel_dev_config->reset_gpio_num;
    ek79007->flags.reset_level = panel_dev_config->flags.reset_active_high;

    // Create MIPI DPI panel
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_dpi(vendor_config->mipi_config.dsi_bus, vendor_config->mipi_config.dpi_config, ret_panel), err, PANEL_TAG,
                      "create MIPI DPI panel failed");
    ESP_LOGD(PANEL_TAG, "new MIPI DPI panel @%p", *ret_panel);

    // Save the original functions of MIPI DPI panel
    ek79007->del = (*ret_panel)->del;
    ek79007->init = (*ret_panel)->init;
    // Overwrite the functions of MIPI DPI panel
    (*ret_panel)->del = panel_ek79007_del;
    (*ret_panel)->init = panel_ek79007_init;
    (*ret_panel)->reset = panel_ek79007_reset;
    (*ret_panel)->mirror = panel_ek79007_mirror;
    (*ret_panel)->invert_color = panel_ek79007_invert_color;
    (*ret_panel)->disp_on_off = panel_ek79007_disp_on_off;
    (*ret_panel)->user_data = ek79007;
    ESP_LOGD(PANEL_TAG, "new ek79007 panel @%p", ek79007);

    return ESP_OK;

err:
    if (ek79007) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(ek79007);
    }
    return ret;
}

static const ek79007_lcd_init_cmd_t vendor_specific_init_default[] = {
//  {cmd, { data }, data_size, delay_ms}
    {0x80, (uint8_t []){0x8B}, 1, 0},
    {0x81, (uint8_t []){0x78}, 1, 0},
    {0x82, (uint8_t []){0x84}, 1, 0},
    {0x83, (uint8_t []){0x88}, 1, 0},
    {0x84, (uint8_t []){0xA8}, 1, 0},
    {0x85, (uint8_t []){0xE3}, 1, 0},
    {0x86, (uint8_t []){0x88}, 1, 0},
    {0xB2, (uint8_t []){0x10}, 1, 0},
    {0x11, (uint8_t []){0x00}, 0, 120},
};

static esp_err_t panel_ek79007_send_init_cmds(ek79007_panel_t *ek79007)
{
    esp_lcd_panel_io_handle_t io = ek79007->io;
    const ek79007_lcd_init_cmd_t *init_cmds = NULL;
    uint16_t init_cmds_size = 0;

    // vendor specific initialization, it can be different between manufacturers
    // should consult the LCD supplier for initialization sequence code
    if (ek79007->init_cmds) {
        init_cmds = ek79007->init_cmds;
        init_cmds_size = ek79007->init_cmds_size;
    } else {
        init_cmds = vendor_specific_init_default;
        init_cmds_size = sizeof(vendor_specific_init_default) / sizeof(ek79007_lcd_init_cmd_t);
    }

    for (int i = 0; i < init_cmds_size; i++) {
        // Send command
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, init_cmds[i].cmd, init_cmds[i].data, init_cmds[i].data_bytes),
                            PANEL_TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(init_cmds[i].delay_ms));
    }
    ESP_LOGD(PANEL_TAG, "send init commands success");

    return ESP_OK;
}

static esp_err_t panel_ek79007_del(esp_lcd_panel_t *panel)
{
    ek79007_panel_t *ek79007 = (ek79007_panel_t *)panel->user_data;

    if (ek79007->reset_gpio_num >= 0) {
        gpio_reset_pin(ek79007->reset_gpio_num);
    }
    // Delete MIPI DPI panel
    ek79007->del(panel);
    free(ek79007);
    ESP_LOGD(PANEL_TAG, "del ek79007 panel @%p", ek79007);

    return ESP_OK;
}

static esp_err_t panel_ek79007_init(esp_lcd_panel_t *panel)
{
    ek79007_panel_t *ek79007 = (ek79007_panel_t *)panel->user_data;

    ESP_RETURN_ON_ERROR(panel_ek79007_send_init_cmds(ek79007), PANEL_TAG, "send init commands failed");
    ESP_RETURN_ON_ERROR(ek79007->init(panel), PANEL_TAG, "init MIPI DPI panel failed");

    return ESP_OK;
}

static esp_err_t panel_ek79007_reset(esp_lcd_panel_t *panel)
{
    ek79007_panel_t *ek79007 = (ek79007_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io = ek79007->io;

    // Perform hardware reset
    if (ek79007->reset_gpio_num >= 0) {
        gpio_set_level(ek79007->reset_gpio_num, ek79007->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(ek79007->reset_gpio_num, !ek79007->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(20));
    } else if (io) { // Perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), PANEL_TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return ESP_OK;
}

static esp_err_t panel_ek79007_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    ek79007_panel_t *ek79007 = (ek79007_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io = ek79007->io;
    uint8_t madctl_val = 0x01;

    ESP_RETURN_ON_FALSE(io, ESP_ERR_INVALID_STATE, PANEL_TAG, "invalid panel IO");

    // Control mirror through LCD command
    if (mirror_x) {
        madctl_val |= EK79007_CMD_SHLR_BIT;
    } else {
        madctl_val &= ~EK79007_CMD_SHLR_BIT;
    }
    if (mirror_y) {
        madctl_val |= EK79007_CMD_UPDN_BIT;
    } else {
        madctl_val &= ~EK79007_CMD_UPDN_BIT;
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t []) {
        madctl_val
    }, 1), PANEL_TAG, "send command failed");

    return ESP_OK;
}

static esp_err_t panel_ek79007_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    ek79007_panel_t *ek79007 = (ek79007_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io = ek79007->io;
    uint8_t command = 0;

    ESP_RETURN_ON_FALSE(io, ESP_ERR_INVALID_STATE, PANEL_TAG, "invalid panel IO");

    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), PANEL_TAG, "send command failed");

    return ESP_OK;
}

static esp_err_t panel_ek79007_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    ESP_LOGE(PANEL_TAG, "display on/off is not supported");

    return ESP_ERR_NOT_SUPPORTED;
}

#endif // SOC_MIPI_DSI_SUPPORTED
