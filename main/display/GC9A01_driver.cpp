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

#define SPI_SPEED_HZ 40 * 1000 * 1000



// LVGL display handles
static lv_display_t* disp_left = nullptr;
static lv_display_t* disp_right = nullptr;

// ESP-IDF LCD panel handles
static esp_lcd_panel_handle_t panel_handle_left = nullptr;
static esp_lcd_panel_handle_t panel_handle_right = nullptr;

// LVGL flush callback for v9
static void lvgl_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);

    int offsetx1 = area->x1;
    int offsety1 = area->y1;
    int offsetx2 = area->x2;
    int offsety2 = area->y2;

    // Apply the software workaround for the G-B color swap issue.
    lv_color_t* color_map = (lv_color_t*)px_map;
    // size_t pixel_count = (offsetx2 - offsetx1 + 1) * (offsety2 - offsety1 + 1);
    // for (size_t i = 0; i < pixel_count; i++) {
    //     // Unpack the pixel into R,G,B components
    //     uint32_t color_int = lv_color_to_u32(color_map[i]);
    //     uint8_t r = (color_int >> 16) & 0xFF;
    //     uint8_t g = (color_int >> 8) & 0xFF;
    //     uint8_t b = color_int & 0xFF;
    //     // Re-pack with G and B swapped
    //     color_map[i] = lv_color_make(r, b, g);
    // }

    // Copy the buffer's content to the specific area of the display.
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
    
    // Notify LVGL that flushing is done.
    lv_display_flush_ready(disp);
}

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
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI bus initialized.");

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

    const esp_lcd_panel_dev_config_t panel_config_left = {
        .reset_gpio_num = PIN_NUM_RST_LEFT,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB, // Set to standard RGB
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
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
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB, // Set to standard RGB
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle_right, &panel_config_right, &panel_handle_right));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle_right));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle_right));
    // Match the working example's settings
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle_right, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle_right, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle_right, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle_right, true));
    ESP_LOGI(TAG, "Right screen initialized.");

    // --- Register Left Screen with LVGL ---
    ESP_LOGI(TAG, "Registering left screen with LVGL...");
    disp_left = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_flush_cb(disp_left, lvgl_flush_cb);
    lv_display_set_user_data(disp_left, panel_handle_left);
    lv_color_t* buf_left = (lv_color_t*)heap_caps_malloc(LCD_H_RES * 10 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_color_t* buf_left_sec = (lv_color_t*)heap_caps_malloc(LCD_H_RES * 10 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf_left);
    lv_display_set_buffers(disp_left, buf_left, buf_left_sec, LCD_H_RES * 10 * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
    ESP_LOGI(TAG, "Left screen registered.");

    // --- Register Right Screen with LVGL ---
    ESP_LOGI(TAG, "Registering right screen with LVGL...");
    disp_right = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_flush_cb(disp_right, lvgl_flush_cb);
    lv_display_set_user_data(disp_right, panel_handle_right);
    lv_color_t* buf_right = (lv_color_t*)heap_caps_malloc(LCD_H_RES * 10 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_color_t* buf_right_sec = (lv_color_t*)heap_caps_malloc(LCD_H_RES * 10 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf_right);
    lv_display_set_buffers(disp_right, buf_right, buf_right_sec, LCD_H_RES * 10 * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
    ESP_LOGI(TAG, "Right screen registered.");

    return true;
}

lv_display_t* get_left_screen_display(void) {
    return disp_left;
}

lv_display_t* get_right_screen_display(void) {
    return disp_right;
}
