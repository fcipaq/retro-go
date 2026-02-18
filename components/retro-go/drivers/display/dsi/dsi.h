/*
 * Snes9x running on the ESP32-P4-Function-EV-Board
 *
 * This file contains some code from: https://github.com/espressif
 * /esp-idf/blob/master/examples/peripherals/lcd/mipi_dsi/main/mipi_dsi_lcd_example_main.c
 *
 * Copyright (C) 2025 Daniel Kammer (daniel.kammer@web.de)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#define LCD_ACCESS_MODE   1 // 0=Windowed transactions, 1=Direct full framebuffer

#include "driver/ledc.h"

#include "esp_cache.h"

#include "esp_log.h"

#include "esp_ldo_regulator.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "driver/ppa.h"

#if RG_PICO_VERSION==3
#include "esp_lcd_hd395003c30.h"
#elif RG_PICO_VERSION==2
#include "esp_lcd_dxq3d9502.h"
#endif
//#include "esp_lcd_ek79007.h"

#define MIPI_DSI_PHY_PWR_LDO_CHAN (3)
#define PPA_SCALER

#define LCD_BUFFER_LENGTH (RG_SCREEN_WIDTH * RG_SCREEN_HEIGHT)

bool m_init = false;

// MIPI driver
static char *TAG = "DISP_DRV";
static esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
static esp_lcd_panel_io_handle_t mipi_dbi_io = NULL;

// framebuffer
#define NUM_FB (3)

volatile int m_scale_factor;

static uint16_t *lcdbuf[NUM_FB]={};  // internal buffers
static uint16_t *rg_lcd_buf;         // interface buffer with RG system

volatile int mode_write_through = 0;

volatile int cur_buf = 1;

static SemaphoreHandle_t drawing_mux;
static TaskHandle_t draw_task_handle;

volatile uint16_t* fb_direct;
volatile uint16_t fb_direct_width;
volatile uint16_t fb_direct_height;

#ifdef SOFTWARE_SCALER
uint16_t linebuffer[LCD_H_RES];
#endif

/* ==================================================================================================== */
static void lcd_write_direct(uint16_t* buffer, int width, int height) {
  if (!buffer || !width || !height)
    return;

  xSemaphoreTake(drawing_mux, portMAX_DELAY);
  fb_direct_width = width;
  fb_direct_height = height;
  fb_direct = buffer;
  xSemaphoreGive(drawing_mux);
 	xTaskNotifyGive(draw_task_handle);
}

static void lcd_write_through(uint16_t* buffer, int width, int height) {
  xSemaphoreTake(drawing_mux, portMAX_DELAY);

  int scale = 3; //LCD_V_RES / width + 1;

  if (!mode_write_through) {
    mode_write_through = 1;
    for (int i = 0; i < NUM_FB; i++)
      for (int j = 0; j < LCD_H_RES * LCD_V_RES; j++)
        lcdbuf[i][j] = 0;
  }

#ifdef PPA_SCALER
  //use ppa to scale image
  ppa_client_config_t ppa_cfg={
		.oper_type=PPA_OPERATION_SRM,
	};

	ppa_client_handle_t ppa;

	ESP_ERROR_CHECK(ppa_register_client(&ppa_cfg, &ppa));

  // TODO: some magic to make it fit the screen

  ppa_srm_oper_config_t op={
    .in={
      .buffer=buffer,
      .pic_w=width,
      .pic_h=height,
      .block_w=240,
      .block_h=224,
      .block_offset_x=8,
      .block_offset_y=0,
      .srm_cm=PPA_SRM_COLOR_MODE_RGB565,
    },
    .out={
      .buffer=lcdbuf[cur_buf],
      .buffer_size=LCD_V_RES*LCD_H_RES*sizeof(int16_t),
      .pic_w=LCD_H_RES,
      .pic_h=LCD_V_RES,
      .block_offset_x=8 * scale,
      .srm_cm=PPA_SRM_COLOR_MODE_RGB565,
    },
    .scale_x=scale,
    .scale_y=scale,
    .rotation_angle=PPA_SRM_ROTATION_ANGLE_270,
    .mode=PPA_TRANS_MODE_BLOCKING,
  };
  ESP_ERROR_CHECK(ppa_do_scale_rotate_mirror(ppa, &op));

  ppa_unregister_client(ppa);
#endif

#ifdef SOFTWARE_SCALER
  // software scaler
  uint16_t* dst2 = lcdbuf[cur_buf];

  for (int y = 0; y < 240; y++) {
  
    uint16_t* src = buffer + 240 * 256 - 8 - (240 - y);
    uint16_t* dst = linebuffer;

      for (int x = 0; x < 240; x++) {
        *dst = *src;
        dst++;

        *dst = *src;
        dst++;

        *dst = *src;
        dst++;

        src -= 256;
      }

      memcpy(dst2, linebuffer, LCD_H_RES * 2);
      dst2 += LCD_H_RES;
      memcpy(dst2, linebuffer, LCD_H_RES * 2);
      dst2 += LCD_H_RES;
      memcpy(dst2, linebuffer, LCD_H_RES * 2);
      dst2 += LCD_H_RES;

  }
#endif

  //do a draw to trigger fb flip
  esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, lcdbuf[cur_buf]);

  cur_buf = (cur_buf + 1) % NUM_FB;
  
  xSemaphoreGive(drawing_mux);

}

static void lcd_draw_direct_task(void *param) {
	while (1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    lcd_write_through(fb_direct, fb_direct_width, fb_direct_height);
  }
}

static void lcd_flip_buffer() {
  xSemaphoreTake(drawing_mux, portMAX_DELAY);

  mode_write_through = 0;

  ppa_client_config_t ppa_cfg={
		.oper_type=PPA_OPERATION_SRM,
	};

	ppa_client_handle_t ppa;

	ESP_ERROR_CHECK(ppa_register_client(&ppa_cfg, &ppa));

  int64_t start_us = rg_system_timer();

  int scale = 2;

  //use ppa to scale image
  ppa_srm_oper_config_t op={
    .in={
      .buffer=rg_lcd_buf,
      .pic_w=RG_SCREEN_WIDTH,
      .pic_h=RG_SCREEN_HEIGHT,
      .block_w=RG_SCREEN_WIDTH,
      .block_h=RG_SCREEN_HEIGHT,
      .srm_cm=PPA_SRM_COLOR_MODE_RGB565,
    },
    .out={
      .buffer=lcdbuf[cur_buf],
      .buffer_size=LCD_V_RES*LCD_H_RES*sizeof(int16_t),
      .pic_w=LCD_H_RES,
      .pic_h=LCD_V_RES,
      .srm_cm=PPA_SRM_COLOR_MODE_RGB565,
    },
    .scale_x=2, //(float)BSP_LCD_H_RES/(float)QUAKEGENERIC_RES_X,
    .scale_y=2, //(float)BSP_LCD_V_RES/(float)QUAKEGENERIC_RES_Y,
    .rotation_angle=PPA_SRM_ROTATION_ANGLE_270,
    .mode=PPA_TRANS_MODE_BLOCKING,
  };
  ESP_ERROR_CHECK(ppa_do_scale_rotate_mirror(ppa, &op));

  //do a draw to trigger fb flip
  esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, lcdbuf[cur_buf]);

  cur_buf = (cur_buf + 1) % NUM_FB;
  
  ppa_unregister_client(ppa);

  int64_t end_us = rg_system_timer();
#if 0
  //Shows the maximum FPS possible given the lcd frame drawing code
  printf("LCD Fps: %02f\n", 1000000.0/(end_us-start_us));
#endif

  xSemaphoreGive(drawing_mux);

}

void lcd_init() {
  if (m_init)
    return;

  //assert(gpio_reset_pin(RG_GPIO_LCD_TE) == ESP_OK);
  //gpio_set_direction(RG_GPIO_LCD_TE, GPIO_MODE_INPUT);

#ifdef RG_GPIO_LCD_BCKL
  ESP_LOGI(TAG, "Turn on LCD backlight");

  gpio_reset_pin((gpio_num_t) RG_GPIO_LCD_BCKL);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = (100000),
        .clk_cfg          = LEDC_USE_XTAL_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = RG_GPIO_LCD_BCKL,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 5, // max: 255, max_battery: 127
        .hpoint         = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_LOGI(TAG, "Backlight init complete");
#endif

#ifdef MIPI_DSI_PHY_PWR_LDO_CHAN
  ESP_LOGI(TAG, "MIPI DSI PHY Powered on");
  esp_ldo_channel_config_t ldo_mipi_phy_config = {
    .chan_id = MIPI_DSI_PHY_PWR_LDO_CHAN,
    .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
  };
  assert(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy) == ESP_OK);
#endif

  ESP_LOGI(TAG, "Initialize MIPI DSI bus");
  esp_lcd_dsi_bus_config_t bus_config = PANEL_BUS_DSI_2CH_CONFIG();
  assert(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus) == ESP_OK);

  ESP_LOGI(TAG, "Install panel IO");
  esp_lcd_dbi_io_config_t dbi_config = PANEL_IO_DBI_CONFIG();
    assert(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &mipi_dbi_io) == ESP_OK);

  ESP_LOGI(TAG, "Install LCD driver of %s", PANEL_TAG);
  esp_lcd_dpi_panel_config_t dpi_config = PANEL_60HZ_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
  dpi_config.num_fbs = NUM_FB;
  panel_vendor_config_t vendor_config = {
    .mipi_config = {
      .dsi_bus = mipi_dsi_bus,
      .dpi_config = &dpi_config,
    },
    .flags = {
      .use_mipi_interface = 1,
    },
  };

  const esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = RG_GPIO_LCD_RST,
    .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
    .bits_per_pixel = 16,
    .vendor_config = &vendor_config,
    .flags = { .reset_active_high = 0 }, 
  };

  assert(esp_lcd_new_panel(mipi_dbi_io, &panel_config, &panel_handle) == ESP_OK);

#if 0
  esp_lcd_dpi_panel_event_callbacks_t cbs = {
    .on_refresh_done = notify_refresh_ready,
  };
  assert(rg_lcd_dpi_panel_register_event_callbacks(panel_handle, &cbs, (void*) &mipi_status_info) == ESP_OK);
#endif

  assert(esp_lcd_panel_reset(panel_handle) == ESP_OK);

  assert(esp_lcd_panel_init(panel_handle) == ESP_OK);

  #ifdef RG_GPIO_PWR_EN
  gpio_set_direction(RG_GPIO_PWR_EN, GPIO_MODE_OUTPUT);
  gpio_set_level(RG_GPIO_PWR_EN, 1);
  #endif

  #ifdef RG_GPIO_LCD_BCKL_EN
  gpio_set_direction(RG_GPIO_LCD_BCKL_EN, GPIO_MODE_OUTPUT);
  gpio_set_level(RG_GPIO_LCD_BCKL_EN, 1);
  #endif

	rg_lcd_buf = heap_caps_calloc(RG_SCREEN_WIDTH * RG_SCREEN_HEIGHT, sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
  assert(rg_lcd_buf);

	drawing_mux = xSemaphoreCreateMutex();
  assert(drawing_mux);

	ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(panel_handle, 3, (void**)&lcdbuf[0], (void**)&lcdbuf[1], (void**)&lcdbuf[2]));

	xTaskCreatePinnedToCore(lcd_draw_direct_task, "draw direct", 4096, NULL, 3, &draw_task_handle, 1);

  m_init = true;
}

//================================================ RG system glue ==============================================
static inline uint16_t *lcd_get_buffer_ptr(int left, int top) {
  return &rg_lcd_buf[left + top * RG_SCREEN_WIDTH];
}

static void lcd_set_backlight(float percent) {
  if (!m_init)
    return;

#ifdef RG_GPIO_LCD_BCKL
  int level = percent * 2.55 / 5.;  // fcipaq TODO debug
  
  if ((level < 0) || (level > 255))
    return;

  // ledcWrite(1 /* PWM channel */, level /* duty cycle */);
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, level));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
#endif
}

static void lcd_deinit(void)
{
  if (!m_init)
    return;

  lcd_set_backlight(0);
  //vTaskDelay(pdMS_TO_TICKS(100));

#ifdef RG_SCREEN_DEINIT
    RG_SCREEN_DEINIT();
#endif

  // TODO: kill draw task

  assert(esp_lcd_panel_del(panel_handle) == ESP_OK);
  assert(esp_lcd_panel_io_del(mipi_dbi_io) == ESP_OK);
  assert(esp_lcd_del_dsi_bus(mipi_dsi_bus) == ESP_OK);
  panel_handle = NULL;
  mipi_dbi_io = NULL;
  mipi_dsi_bus = NULL;

  if (ldo_mipi_phy) {
    assert(esp_ldo_release_channel(ldo_mipi_phy) == ESP_OK);
    ldo_mipi_phy = NULL;
  }

  assert(gpio_reset_pin(RG_GPIO_LCD_RST) == ESP_OK);
  gpio_set_direction(RG_GPIO_LCD_RST, GPIO_MODE_OUTPUT);
  gpio_set_level(RG_GPIO_LCD_RST, 0);

  #ifdef RG_GPIO_LCD_BCKL
  assert(gpio_reset_pin(RG_GPIO_LCD_BCKL) == ESP_OK);
  #endif

  free(rg_lcd_buf);

  m_init = false;
}

static void lcd_sync(void)
{
  lcd_flip_buffer();
}

const rg_display_driver_t rg_display_driver_dsi = {
    .name = "dsi",
};
