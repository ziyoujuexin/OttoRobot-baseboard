#ifndef GC9A01_DRIVER_H
#define GC9A01_DRIVER_H

#include "lvgl.h"

// Pin definitions for the GC9A01 screens
// Please adjust these to your hardware configuration
#define LCD_HOST            SPI2_HOST

// Shared PINS
#define PIN_NUM_SCLK        49
#define PIN_NUM_MOSI        50
#define PIN_NUM_DC          2

// Left Screen PINS
#define PIN_NUM_CS_LEFT     3
#define PIN_NUM_RST_LEFT    7
#define PIN_NUM_BL_LEFT     25 // Backlight

// Right Screen PINS
#define PIN_NUM_CS_RIGHT    4
#define PIN_NUM_RST_RIGHT   8
#define PIN_NUM_BL_RIGHT    24 // Backlight

#define LCD_H_RES           240
#define LCD_V_RES           240
#define LCD_BIT_PER_PIXEL   16

/**
 * @brief Initialize the two GC9A01 screens and register them with LVGL.
 *
 * @return true if initialization is successful, false otherwise.
 */
bool gc9a01_lvgl_driver_init(void);

/**
 * @brief Enable or disable mirrored display mode.
 *
 * When enabled, the content of the left screen is mirrored to the right screen.
 *
 * @param enabled True to enable mirror mode, false to disable.
 */
void set_mirror_mode(bool enabled);

/**
 * @brief Get the LVGL display object for the left screen.
 *
 * @return Pointer to the left screen's lv_disp_t object.
 */
lv_display_t* get_left_screen_display(void);

/**
 * @brief Get the LVGL display object for the right screen.
 *
 * @return Pointer to the right screen's lv_display_t object.
 */
lv_display_t* get_right_screen_display(void);


#endif // GC9A01_DRIVER_H
