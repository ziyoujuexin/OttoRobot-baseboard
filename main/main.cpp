#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <memory> // Required for std::unique_ptr
#include "freertos/queue.h"

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

static const char *TAG = "MAIN";

// --- LVGL & UI Command Queue Setup ---
static SemaphoreHandle_t lvgl_mutex;

// Define the command structure for UI updates
struct UiCommand {
    char animation_name[32];
};

// Declare the queue handle
QueueHandle_t ui_command_queue;

// Structure to pass multiple managers to the LVGL task
struct LvglTaskParams {
    DualScreenManager* display_manager;
    AnimationManager* animation_manager;
};


// A FreeRTOS task to run the LVGL timer handler and process UI commands.
static void lvgl_task(void *pvParameter) {
    ESP_LOGI(TAG, "Starting LVGL task");
    LvglTaskParams* params = (LvglTaskParams*)pvParameter;
    DualScreenManager* display_manager = params->display_manager;
    AnimationManager* animation_manager = params->animation_manager;
    
    uint32_t task_delay_ms = 10;
    AnimationData current_anim_data; // Holds the currently loaded animation data

    // Variables for performance monitoring
    uint32_t last_handler_time = 0;
    uint32_t max_diff = 0;
    uint32_t min_diff = UINT32_MAX;
    uint64_t total_diff = 0;
    uint32_t sample_count = 0;

    while(1) {
        // 1. Check and process UI commands (non-blocking)
        UiCommand received_cmd;
        if (xQueueReceive(ui_command_queue, &received_cmd, 0) == pdPASS) {
            ESP_LOGI(TAG, "LVGL task received command to play: %s", received_cmd.animation_name);
            
            if (xSemaphoreTake(lvgl_mutex, portMAX_DELAY)) {
                // 1. Release previous animation data if it's valid
                if (current_anim_data.is_valid) {
                    animation_manager->releaseAnimationData(current_anim_data);
                }

                // 2. Get new animation data from the provider
                current_anim_data = animation_manager->getAnimationData(received_cmd.animation_name);

                // 3. Update the display with the new data (from memory)
                display_manager->UpdateAnimationSource(current_anim_data);

                xSemaphoreGive(lvgl_mutex);
            }
        }

        // 2. Standard LVGL handler loop
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
        if (xSemaphoreTake(lvgl_mutex, portMAX_DELAY)) {
            // --- Performance Measurement ---
            uint32_t current_time = lv_tick_get();
            if (last_handler_time != 0) {
                uint32_t time_diff = current_time - last_handler_time;
                if (time_diff > max_diff) max_diff = time_diff;
                if (time_diff < min_diff) min_diff = time_diff;
                total_diff += time_diff;
                sample_count++;
            }
            last_handler_time = current_time;
            // --- End Measurement ---

            lv_tick_inc(task_delay_ms);
            lv_timer_handler();
            xSemaphoreGive(lvgl_mutex);
        }
        
        static int counter = 0;
        if (++counter % 200 == 0) { // Every second
            if (sample_count > 0) {
                uint32_t avg_diff = total_diff / sample_count;
                ESP_LOGI(TAG, "LVGL Handler Timing (ms) - Avg: %d, Min: %d, Max: %d", avg_diff, min_diff, max_diff);
                // Reset stats
                max_diff = 0;
                min_diff = UINT32_MAX;
                total_diff = 0;
                sample_count = 0;
            }
            ESP_LOGI(TAG, "LVGL Task Stack High Water Mark: %u bytes", uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
            counter = 0;
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
        while(1) { vTaskDelay(1000); }
    }

    // Initialize LVGL and the display driver
    lv_init();
    lvgl_mutex = xSemaphoreCreateMutex();
    ui_command_queue = xQueueCreate(10, sizeof(UiCommand)); // Create the UI command queue
    lvgl_fs_driver_init();
    if (!gc9a01_lvgl_driver_init()) {
        ESP_LOGE(TAG, "Failed to initialize display driver. Halting.");
        while(1) { vTaskDelay(1000); }
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
    
    // The AnimationPlayer no longer needs the mutex
    auto animation_player = std::make_unique<AnimationPlayer>(animation_manager.get(), display_manager.get());
    animation_player->start();

    // Create the LVGL task and pass necessary managers to it
    static LvglTaskParams lvgl_params;
    lvgl_params.display_manager = display_manager.get();
    lvgl_params.animation_manager = animation_manager.get();
    xTaskCreate(lvgl_task, "lvgl_task", 4096*16, &lvgl_params, 10, NULL);

    auto face_location_callback = [&](const FaceLocation& loc) {
        if (motion_controller) {
            motion_controller->queue_face_location(loc);
        }
    };
    auto uart_handler = std::make_unique<UartHandler>(*motion_controller, animation_player.get(), face_location_callback);
    uart_handler->init();
    
    auto sound_manager = std::make_unique<SoundManager>(motion_controller.get(), uart_handler.get());
    sound_manager->start();

    auto web_server = std::make_unique<WebServer>(*action_manager, *motion_controller);
    web_server->start();


    // --- 4. Post-Init Actions & Main Loop ---
    ESP_LOGI(TAG, "Phase 4: Post-Initialization and Main Loop");

    while(1) {
        vTaskDelay(portMAX_DELAY);
    }
}
