#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/PCA9685.hpp"
#include "motion_manager/ActionManager.hpp"
#include "MotionController.hpp"
#include "UartHandler.hpp"
#include "web_server/WebServer.hpp"
#include "sound/SoundManager.hpp"
#include <i2cdev.h>
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

static const char *TAG = "MAIN";

// Declare pointers for our long-lived objects
static ActionManager* action_manager_ptr = nullptr;
static MotionController* motion_controller_ptr = nullptr;
static UartHandler* uart_handler_ptr = nullptr;
static WebServer* web_server_ptr = nullptr;
static PCA9685* servo_driver_ptr = nullptr;
static SoundManager* sound_manager_ptr = nullptr;

// Task to react to sound localization
void sound_reaction_task(void* pvParameters) {
    ESP_LOGI(TAG, "Sound reaction task started.");
    int last_processed_angle = -1;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (uart_handler_ptr->m_isWakeWordDetected == false) {
            continue; // Skip processing if wake word not detected
        }
        int detected_angle = sound_manager_ptr->get_last_detected_angle();

        // Only react if a new sound event has been processed
        if (detected_angle != -1 && detected_angle != last_processed_angle) {
            ESP_LOGI(TAG, "New sound detected at angle: %d. Reacting.", detected_angle);
            // Simple logic: turn left or right towards the sound
            // Assuming 0 degrees is forward, 90 is right, 270 is left
            if (detected_angle > 10 && detected_angle < 170) {
                ESP_LOGI(TAG, "Sound from the right, turning right.");
                motion_controller_ptr->queue_command({MOTION_TRACKING_R, {}});
                uart_handler_ptr->m_isWakeWordDetected = false;
                vTaskDelay(pdMS_TO_TICKS(20)); // At least delay 20ms for msg transmission
            } else if (detected_angle > 190 && detected_angle < 350) {
                ESP_LOGI(TAG, "Sound from the left, turning left.");
                motion_controller_ptr->queue_command({MOTION_TRACKING_L, {}});
                uart_handler_ptr->m_isWakeWordDetected = false;
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            last_processed_angle = detected_angle;
            
            while (!motion_controller_ptr->is_idle()) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            ESP_LOGI(TAG, "Sound Reaction complete, stopping.");
            motion_controller_ptr->queue_command({MOTION_STOP, {}});
        }
    }
}


extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Application startup.");
    
    // Initialize I2C bus for peripherals
    i2cdev_init();

    // Create the servo driver on the heap
    servo_driver_ptr = new PCA9685();
    servo_driver_ptr->init();

    // Create the action manager on the heap
    action_manager_ptr = new ActionManager();
    action_manager_ptr->init();

    // Create the motion controller on the heap
    motion_controller_ptr = new MotionController(*servo_driver_ptr, *action_manager_ptr);
    motion_controller_ptr->init();

    // Create the UART command handler on the heap
    // uart_handler_ptr = new UartHandler(*motion_controller_ptr);
    auto face_location_callback = [&](const FaceLocation& loc) {
        if (motion_controller_ptr) {
            motion_controller_ptr->queue_face_location(loc);
        }
    };
    uart_handler_ptr = new UartHandler(face_location_callback);
    uart_handler_ptr->init();
    
    // Create and start the web server on the heap
    web_server_ptr = new WebServer(*action_manager_ptr, *motion_controller_ptr);
    web_server_ptr->start();

    // Create and start the sound manager
    sound_manager_ptr = new SoundManager();
    sound_manager_ptr->start();

    // Create the sound reaction task
    xTaskCreate(sound_reaction_task, "SoundReactionTask", 4096, NULL, 5, NULL);


    ESP_LOGI(TAG, "Initialization complete. All modules started.");

    // alawys keep following code for NVS storage debugging
    // Clean and re-register default actions for debugging
    // action_manager_ptr->delete_action_from_nvs("walk_forward");
    // action_manager_ptr->delete_action_from_nvs("walk_backward");
    // action_manager_ptr->delete_action_from_nvs("turn_left");
    // action_manager_ptr->delete_action_from_nvs("turn_right");
    // action_manager_ptr->delete_action_from_nvs("wiggle_ears");
    // action_manager_ptr->delete_action_from_nvs("wave_hand");
    // action_manager_ptr->delete_action_from_nvs("nod_head");
    // action_manager_ptr->delete_action_from_nvs("shake_head");
    // action_manager_ptr->delete_action_from_nvs("single_leg");
    // action_manager_ptr->register_default_actions();
    // motion_controller_ptr->queue_command({MOTION_STOP, {}});
    
}
