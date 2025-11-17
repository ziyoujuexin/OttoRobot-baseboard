#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_task_wdt.h>
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
#include "AnimationPlayer.h"
#include "ActionManager.hpp"
#include "MotionController.hpp"
#include "SoundManager.hpp"
#include "UartHandler.hpp"
#include "WebServer.hpp"
#include "UIManager.hpp" // Use the new UIManager

#include "esp_sleep.h"

static const char *TAG = "MAIN";

// The LVGL tick hook is a FreeRTOS requirement and can stay global.
void vApplicationTickHook( void ){
    lv_tick_inc(1); // RTOS run 1000Hz
}

// Function to print task stats for debugging
void print_task_stats(void *pvParameters) {
    const char *TAG_STATS = "TASK_STATS";
    
    while (1) {
        // Wait for 10 seconds before printing stats
        vTaskDelay(pdMS_TO_TICKS(10000));

        size_t num_tasks = uxTaskGetNumberOfTasks();
        // Allocate a buffer big enough for the task list.
        // 50 bytes per task is a safe estimate.
        char *buffer = (char*) malloc(num_tasks * 50); 

        if (buffer == NULL) {
            ESP_LOGE(TAG_STATS, "Cannot allocate memory for stats buffer");
            continue; // Try again after delay
        }

        ESP_LOGI(TAG_STATS, "--- FreeRTOS Task List ---");
        vTaskList(buffer);
        printf("%s", buffer); // vTaskList includes newlines

        ESP_LOGI(TAG_STATS, "--- Task Runtime Stats ---");
        vTaskGetRunTimeStats(buffer);
        printf("%s", buffer); // vTaskGetRunTimeStats includes newlines
        
        free(buffer);

        ESP_LOGI(TAG_STATS, "--- End of Stats ---");
    }
}


// --- Main Application --- 

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));//延时错位
    ESP_ERROR_CHECK(i2cdev_init());      // For servo driver

    if (sd_card_manager::init("/sdcard") != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SD card. Halting.");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Initialize LVGL core and filesystem
    lv_init();
    lvgl_fs_driver_init();
    if (!gc9a01_lvgl_driver_init()) {
        ESP_LOGE(TAG, "Failed to initialize display driver. Continuing without display.");
    }
    // --- 3. Application Services and Managers Initialization ---
    ESP_LOGI(TAG, "Phase 3: Initializing Application Services & Managers");

    auto servo_driver = std::make_unique<PCA9685>();
    servo_driver->init();

    auto action_manager = std::make_unique<ActionManager>();
    action_manager->init();

    auto motion_controller = std::make_unique<MotionController>(*servo_driver, *action_manager);
    motion_controller->init();

    auto display_manager = std::make_unique<DualScreenManager>();
    auto sd_provider = std::make_unique<SDCardAnimationProvider>("/sdcard/animations");
    auto animation_manager = std::make_unique<AnimationManager>(std::move(sd_provider));
    
    // Create and initialize the UIManager, which now handles the LVGL task
    auto ui_manager = std::make_unique<UIManager>(display_manager.get(), animation_manager.get());
    ui_manager->init();

    // The AnimationPlayer now also needs the UI queue to post default/state-driven animations
    auto animation_player = std::make_unique<AnimationPlayer>(animation_manager.get(), display_manager.get(), ui_manager->get_command_queue());
    animation_player->start();

    auto face_location_callback = [&](const FaceLocation& loc) {
        if (motion_controller) {
            motion_controller->queue_face_location(loc);
        }
    };
    auto uart_handler = std::make_unique<UartHandler>(*motion_controller, animation_player.get(), face_location_callback);
    uart_handler->init();
    
    auto sound_manager = std::make_unique<SoundManager>(motion_controller.get(), uart_handler.get());
    sound_manager->start();

    // Pass the AnimationPlayer instance to the WebServer
    auto web_server = std::make_unique<WebServer>(*action_manager, *motion_controller, *animation_player);
    web_server->start();

    // Create a task to print task stats for debugging
    xTaskCreate(print_task_stats, "print_task_stats", 4096, NULL, 5, NULL);

    vTaskDelete(NULL); // Delete main task as its work is done
}