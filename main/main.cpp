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

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Application startup.");
    
    // Initialize I2C bus for peripherals
    i2cdev_init();

    servo_driver_ptr = new PCA9685();
    servo_driver_ptr->init();

    action_manager_ptr = new ActionManager();
    action_manager_ptr->init();

    motion_controller_ptr = new MotionController(*servo_driver_ptr, *action_manager_ptr);
    motion_controller_ptr->init();

    // uart_handler_ptr = new UartHandler(*motion_controller_ptr);
    auto face_location_callback = [&](const FaceLocation& loc) {
        if (motion_controller_ptr) {
            motion_controller_ptr->queue_face_location(loc);
        }
    };
    uart_handler_ptr = new UartHandler(*motion_controller_ptr, face_location_callback);
    uart_handler_ptr->init();
    
    // Create and start the web server on the heap
    web_server_ptr = new WebServer(*action_manager_ptr, *motion_controller_ptr);
    web_server_ptr->start();

    // Create and start the sound manager
    sound_manager_ptr = new SoundManager(motion_controller_ptr, uart_handler_ptr);
    sound_manager_ptr->start();

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