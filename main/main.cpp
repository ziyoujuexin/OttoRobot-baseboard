#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <memory> // Required for std::unique_ptr

// Core Drivers & Services
#include <i2cdev.h>
#include "nvs_flash.h"
#include "driver/sd_card_manager.h"
#include "driver/PCA9685.hpp"

// LVGL & Display
#include "lvgl.h"
#include "GC9A01_driver.hpp"
#include "lvgl_fs_port.h"

// Application-level Managers
#include "DualScreenManager.h"
#include "SDCardAnimationProvider.h"
#include "AnimationManager.h"
#include "ActionManager.hpp"
#include "MotionController.hpp"
#include "SoundManager.hpp"
#include "UartHandler.hpp"
#include "WebServer.hpp"

static const char *TAG = "MAIN";

// --- LVGL Minimal Setup ---
static SemaphoreHandle_t lvgl_mutex;

// A FreeRTOS task to run the LVGL timer handler periodically.
static void lvgl_task(void *pvParameter) {
    ESP_LOGI(TAG, "Starting LVGL task");
    while(1) {
        // This is the heartbeat of LVGL. It handles animations, events, etc.
        vTaskDelay(pdMS_TO_TICKS(10));
        if (xSemaphoreTake(lvgl_mutex, portMAX_DELAY)) {
            lv_tick_inc(10);
            lv_timer_handler();
            xSemaphoreGive(lvgl_mutex);
        }
    }
}

// --- Main Application --- 

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init()); // For WiFi credentials
    ESP_ERROR_CHECK(i2cdev_init());      // For servo driver

    if (sd_card_manager::init("/sdcard") != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SD card. Halting.");
        // In a real product, you might want to enter a safe mode instead of halting.
        while(1) { vTaskDelay(1000); }
    }

    // Initialize LVGL and the display driver
    lv_init();
    lvgl_mutex = xSemaphoreCreateMutex();
    // lv_fs_stdio_init();
    lvgl_fs_driver_init();
    if (!gc9a01_lvgl_driver_init()) {
        ESP_LOGE(TAG, "Failed to initialize display driver. Halting.");
        while(1) { vTaskDelay(1000); }
    }

    

    // Create the FreeRTOS task for the LVGL handler
    xTaskCreate(lvgl_task, "lvgl_task", 4096*2, NULL, 5, NULL);



    // --- 3. Application Services and Managers Initialization ---
    ESP_LOGI(TAG, "Phase 3: Initializing Application Services & Managers");

    // Using std::unique_ptr for automatic memory management.
    auto servo_driver = std::make_unique<PCA9685>();
    servo_driver->init();

    auto action_manager = std::make_unique<ActionManager>();
    action_manager->init();

    auto motion_controller = std::make_unique<MotionController>(*servo_driver, *action_manager);
    motion_controller->init();

    auto display_manager = std::make_unique<DualScreenManager>();
    auto sd_provider = std::make_unique<SDCardAnimationProvider>("/sdcard/animations");
    auto animation_manager = std::make_unique<AnimationManager>(std::move(sd_provider), display_manager.get());

    auto face_location_callback = [&](const FaceLocation& loc) {
        if (motion_controller) {
            motion_controller->queue_face_location(loc);
        }
    };
    auto uart_handler = std::make_unique<UartHandler>(*motion_controller, face_location_callback);
    uart_handler->init();
    
    auto sound_manager = std::make_unique<SoundManager>(motion_controller.get(), uart_handler.get());
    sound_manager->start();

    // The WebServer starts WiFi and the HTTP server internally.
    auto web_server = std::make_unique<WebServer>(*action_manager, *motion_controller);
    web_server->start();


    // --- 4. Post-Init Actions & Main Loop ---
    ESP_LOGI(TAG, "Phase 4: Post-Initialization and Main Loop");

    // Example: Play a boot-up animation
    // if (animation_manager) {
    //     ESP_LOGI(TAG, "Playing rainbow animation...");
    //     animation_manager->PlayAnimation("rainbow", SCREEN_BOTH);
    // }
    while(1) {
        // vTaskDelay(pdMS_TO_TICKS(5000));

        // Example of playing animations periodically in the main loop
        // ESP_LOGI(TAG, "Heap before playing anim: %d bytes", esp_get_free_heap_size());
        // if (animation_manager) {
        //     ESP_LOGI(TAG, "Playing 'jump' animation in main loop...");
        //     animation_manager->PlayAnimation("jump", SCREEN_LEFT);
        // }

        vTaskDelay(pdMS_TO_TICKS(5000));
        // if (animation_manager) {
        //     ESP_LOGI(TAG, "Playing 'jump' animation in main loop...");
        //     animation_manager->PlayAnimation("jump", SCREEN_RIGHT);
        // }
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (animation_manager) {
            ESP_LOGI(TAG, "Playing 'bird' animation in main loop...");
            if (xSemaphoreTake(lvgl_mutex, portMAX_DELAY)) {
                animation_manager->PlayAnimation("bird", SCREEN_BOTH);
                xSemaphoreGive(lvgl_mutex);
            }
        }
        else {
            ESP_LOGW(TAG, "AnimationManager not available in main loop.");
        }
    }
}
