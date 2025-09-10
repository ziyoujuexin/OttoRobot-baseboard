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
    ESP_LOGI(TAG, "SOUND TEST MODE. Application startup.");
    
    // Initialize I2C bus for peripherals
    i2cdev_init();

    servo_driver_ptr = new PCA9685();
    servo_driver_ptr->init();

    action_manager_ptr = new ActionManager();
    action_manager_ptr->init();

    motion_controller_ptr = new MotionController(*servo_driver_ptr, *action_manager_ptr);
    motion_controller_ptr->init();

    auto face_location_callback = [&](const FaceLocation& loc) {
        if (motion_controller_ptr) {
            motion_controller_ptr->queue_face_location(loc);
        }
    };
    uart_handler_ptr = new UartHandler(*motion_controller_ptr, face_location_callback);
    uart_handler_ptr->init();
    
    web_server_ptr = new WebServer(*action_manager_ptr, *motion_controller_ptr);
    web_server_ptr->start();

    // Create and start the sound manager
    sound_manager_ptr = new SoundManager(motion_controller_ptr, uart_handler_ptr);
    sound_manager_ptr->start();


    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "Main task heartbeat...");
    }
}