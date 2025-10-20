#include "GC9A01_driver.hpp"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_gc9a01.h"
#include "esp_heap_caps.h"
#include <esp_cache.h> // For cache synchronization

static const char* TAG = "GC9A01_driver";

#define SPI_SPEED_HZ 80 * 1000 * 1000



static bool g_mirror_mode_enabled = false;

void set_mirror_mode(bool enabled) {
    g_mirror_mode_enabled = enabled;
    ESP_LOGI(TAG, "Mirror mode %s", enabled ? "enabled" : "disabled");
}

// LVGL display handles
static lv_display_t* disp_left = nullptr;
static lv_display_t* disp_right = nullptr;

// ESP-IDF LCD panel handles
static esp_lcd_panel_handle_t panel_handle_left = nullptr;
static esp_lcd_panel_handle_t panel_handle_right = nullptr;

static bool panel_flush_ready(esp_lcd_panel_io_t* io, esp_lcd_panel_io_event_data_t* edata, void* user_ctx) {
    lv_display_t* disp = (lv_display_t*)user_ctx;
    lv_display_flush_ready(disp);
    return false; // No need for a context switch
}

// LVGL flush callback for v9
static void lvgl_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);

    int offsetx1 = area->x1;
    int offsety1 = area->y1;
    int offsetx2 = area->x2;
    int offsety2 = area->y2;
    lv_draw_sw_rgb565_swap(px_map, (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1));
    // we swap bytes here, due to unknown problems, it doesn't work by lv_display_set_color_format(....)
    // actually, swap it by cpu is the right way because LV_COLOR_16_SWAP do the same thing: 
    // https://github.com/lvgl/lvgl/blob/73df4fc1e984b189b5a8708ad15248b5a475c1d4/src/core/lv_refr.c#L1418

    // Copy the buffer's content to the specific area of the display.
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
    
    if (g_mirror_mode_enabled) {
        esp_lcd_panel_draw_bitmap(panel_handle_right, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
    }
}

static void lvgl_flush_cb_sec(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    if (g_mirror_mode_enabled) {
        // In mirror mode, the left screen's flush callback handles both screens.
        // We just need to notify LVGL that this display is ready for the next frame.
        lv_display_flush_ready(disp);
        return;
    }

    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    int offsetx1 = area->x1;
    int offsety1 = area->y1;
    int offsetx2 = area->x2;
    int offsety2 = area->y2;
    lv_draw_sw_rgb565_swap(px_map, (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1));
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
}

// this custom init parm make color closer to reality
static const gc9a01_lcd_init_cmd_t lcd_init_cmds[] = {
//  {cmd, { data }, data_size, delay_ms}
    // Enable Inter Register
    {0xfe, (uint8_t []){0x00}, 0, 0},
    {0xef, (uint8_t []){0x00}, 0, 0},
    {0xeb, (uint8_t []){0x14}, 1, 0},
    {0x84, (uint8_t []){0x40}, 1, 0},
    {0x85, (uint8_t []){0xf1}, 1, 0},
    {0x86, (uint8_t []){0x98}, 1, 0},
    {0x87, (uint8_t []){0x28}, 1, 0},
    {0x88, (uint8_t []){0x0a}, 1, 0},
    {0x89, (uint8_t []){0x21}, 1, 0},
    {0x8a, (uint8_t []){0x00}, 1, 0},
    {0x8b, (uint8_t []){0x80}, 1, 0},
    {0x8c, (uint8_t []){0x01}, 1, 0},
    {0x8d, (uint8_t []){0x01}, 1, 0},
    {0x8e, (uint8_t []){0xdf}, 1, 0},
    {0x8f, (uint8_t []){0x52}, 1, 0},
    {0xb6, (uint8_t []){0x20}, 1, 0},
    // {0x36, (uint8_t []){0x48}, 1, 0},
    // {0x3a, (uint8_t []){0x05}, 1, 0},
    {0x90, (uint8_t []){0x08, 0x08, 0x08, 0x08}, 4, 0},
    {0xE8, (uint8_t []){0x34}, 1, 0}, // 4 dot inversion
    {0xff, (uint8_t []){0x60, 0x01, 0x04}, 3, 0},
    {0x74, (uint8_t []){0x10, 0x75, 0x80, 0x00, 0x00, 0x4E, 0x00}, 7, 0},
    {0xC3, (uint8_t []){0x14}, 1, 0},
    {0xC4, (uint8_t []){0x14}, 1, 0},
    {0xC9, (uint8_t []){0x25}, 1, 0},
    {0xbe, (uint8_t []){0x11}, 1, 0},
    {0xe1, (uint8_t []){0x10, 0x0e}, 2, 0},
    {0xdf, (uint8_t []){0x21, 0x0c, 0x02}, 3, 0},
    {0xed, (uint8_t []){0x1b, 0x0b}, 2, 0},
    {0xae, (uint8_t []){0x77}, 1, 0},
    {0xcd, (uint8_t []){0x63}, 1, 0},
    {0x70, (uint8_t []){0x07, 0x07, 0x04, 0x0e, 0x0f, 0x09, 0x07, 0x08, 0x03}, 9, 0},
    {0xF0, (uint8_t []){0x46, 0x09, 0x0a, 0x08, 0x05, 0x2c}, 6, 0},
    {0xF1, (uint8_t []){0x46, 0x76, 0x76, 0x32, 0x36, 0x9f}, 6, 0},
    {0xF2, (uint8_t []){0x46, 0x09, 0x0a, 0x08, 0x05, 0x2c}, 6, 0},
    {0xF3, (uint8_t []){0x46, 0x76, 0x76, 0x32, 0x36, 0x9f}, 6, 0},
    {0x62, (uint8_t []){0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70}, 12, 0},
    {0x63, (uint8_t []){0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70}, 12, 0},
    {0x64, (uint8_t []){0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07}, 7, 0},
    {0x66, (uint8_t []){0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00}, 10, 0},
    {0x67, (uint8_t []){0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98}, 10, 0},
    {0x98, (uint8_t []){0x3e, 0x07}, 2, 0},
    {0xba, (uint8_t []){0x80}, 1, 0},
    {0x35, (uint8_t []){}, 0, 0},
    {0x21, (uint8_t []){}, 0, 120},
    {0x11, (uint8_t []){}, 0, 120},
    {0x29, (uint8_t []){}, 0, 20},
};


bool gc9a01_lvgl_driver_init(void) {
    ESP_LOGI(TAG, "Initializing GC9A01 screens with %dHz clock...", SPI_SPEED_HZ);

    // Initialize backlight GPIOs
    gpio_config_t bl_gpio_config = {};
    bl_gpio_config.pin_bit_mask = (1ULL << PIN_NUM_BL_LEFT) | (1ULL << PIN_NUM_BL_RIGHT);
    bl_gpio_config.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&bl_gpio_config));
    gpio_set_level((gpio_num_t)PIN_NUM_BL_LEFT, 1);
    gpio_set_level((gpio_num_t)PIN_NUM_BL_RIGHT, 1);
    ESP_LOGI(TAG, "Backlight pins initialized.");

    // Initialize SPI bus
    const spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 120 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI bus initialized.");

    esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = panel_flush_ready,
    };
    esp_lcd_panel_io_callbacks_t cbs_sec = {
        .on_color_trans_done = panel_flush_ready,
    };

    // --- Initialize Left Screen ---
    ESP_LOGI(TAG, "Initializing left screen...");
    esp_lcd_panel_io_handle_t io_handle_left = NULL;
    const esp_lcd_panel_io_spi_config_t io_config_left = {
        .cs_gpio_num = PIN_NUM_CS_LEFT,
        .dc_gpio_num = PIN_NUM_DC,
        .spi_mode = 0,
        .pclk_hz = SPI_SPEED_HZ,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config_left, &io_handle_left));

    gc9a01_vendor_config_t vendor_config = {  // Uncomment these lines if use custom initialization commands
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
    };

    const esp_lcd_panel_dev_config_t panel_config_left = {
        .reset_gpio_num = PIN_NUM_RST_LEFT,
        .rgb_endian = LCD_RGB_ELEMENT_ORDER_BGR, // Set to standard RGB
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle_left, &panel_config_left, &panel_handle_left));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle_left));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle_left));
    // Match the working example's settings
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle_left, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle_left, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle_left, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle_left, true));
    ESP_LOGI(TAG, "Left screen initialized.");

    // --- Initialize Right Screen ---
    ESP_LOGI(TAG, "Initializing right screen...");
    esp_lcd_panel_io_handle_t io_handle_right = NULL;
    const esp_lcd_panel_io_spi_config_t io_config_right = {
        .cs_gpio_num = PIN_NUM_CS_RIGHT,
        .dc_gpio_num = PIN_NUM_DC,
        .spi_mode = 0,
        .pclk_hz = SPI_SPEED_HZ,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config_right, &io_handle_right));

    const esp_lcd_panel_dev_config_t panel_config_right = {
        .reset_gpio_num = PIN_NUM_RST_RIGHT,
        .rgb_endian = LCD_RGB_ELEMENT_ORDER_BGR, // Set to standard RGB
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle_right, &panel_config_right, &panel_handle_right));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle_right));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle_right));
    // Match the working example's settings
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle_right, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle_right, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle_right, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle_right, true));
    ESP_LOGI(TAG, "Right screen initialized.");

    // --- Register Left Screen with LVGL ---
    ESP_LOGI(TAG, "Registering left screen with LVGL...");
    disp_left = lv_display_create(LCD_H_RES, LCD_V_RES);
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle_left, &cbs, disp_left));
    lv_display_set_flush_cb(disp_left, lvgl_flush_cb);
    lv_display_set_user_data(disp_left, panel_handle_left);
    lv_color_t* buf_left = (lv_color_t*)heap_caps_malloc(LCD_H_RES * 60 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    lv_color_t* buf_left_sec = (lv_color_t*)heap_caps_malloc(LCD_H_RES * 60 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf_left);
    lv_display_set_buffers(disp_left, buf_left, buf_left_sec, LCD_H_RES * 60 * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
    ESP_LOGI(TAG, "Left screen registered.");

    // --- Register Right Screen with LVGL ---
    ESP_LOGI(TAG, "Registering right screen with LVGL...");
    disp_right = lv_display_create(LCD_H_RES, LCD_V_RES);
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle_right, &cbs_sec, disp_right));
    lv_display_set_flush_cb(disp_right, lvgl_flush_cb_sec);
    lv_display_set_user_data(disp_right, panel_handle_right);
    lv_color_t* buf_right = (lv_color_t*)heap_caps_malloc(LCD_H_RES * 60  * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    lv_color_t* buf_right_sec = (lv_color_t*)heap_caps_malloc(LCD_H_RES * 60  * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf_right);
    lv_display_set_buffers(disp_right, buf_right, buf_right_sec, LCD_H_RES * 60  * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
    ESP_LOGI(TAG, "Right screen registered.");
    ESP_LOGW(TAG, "Heap after display init: %d", esp_get_free_heap_size());
    return true;
}

lv_display_t* get_left_screen_display(void) {
    return disp_left;
}

lv_display_t* get_right_screen_display(void) {
    return disp_right;
}
