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

// Cleaned-up Includes
#include "driver/sd_card_manager.h"
#include "display/DualScreenManager.h"
#include "display/SDCardAnimationProvider.h"
#include "display/AnimationManager.h"
#include <memory>

static const char *TAG = "MAIN";

// Declare pointers for our long-lived objects
static ActionManager* action_manager_ptr = nullptr;
static MotionController* motion_controller_ptr = nullptr;
static UartHandler* uart_handler_ptr = nullptr;
static WebServer* web_server_ptr = nullptr;
static PCA9685* servo_driver_ptr = nullptr;
static SoundManager* sound_manager_ptr = nullptr;

// Pointers for the new display system
static DualScreenManager* display_manager_ptr = nullptr;
static AnimationManager* animation_manager_ptr = nullptr;


extern "C" void app_main(void)
{
    i2cdev_init();

    // Initialize SD card using the dedicated manager
    if (sd_card_manager::init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SD card. Animations will not be available.");
        // You might want to halt or use a fallback here
    }

    // Initialize Display System
    ESP_LOGI(TAG, "Initializing Display System...");
    display_manager_ptr = new DualScreenManager();
    auto sd_provider = std::make_unique<SDCardAnimationProvider>("/");
    animation_manager_ptr = new AnimationManager(std::move(sd_provider), display_manager_ptr);
    ESP_LOGI(TAG, "Display System Initialized.");

    // Example: Play a boot-up animation
    if (animation_manager_ptr) {
        ESP_LOGI(TAG, "Playing boot animation...");
        animation_manager_ptr->PlayAnimation("boot", SCREEN_BOTH);
    }

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

    sound_manager_ptr = new SoundManager(motion_controller_ptr, uart_handler_ptr);
    sound_manager_ptr->start();


    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        // Example of playing another animation in the main loop
        if (animation_manager_ptr) {
            ESP_LOGI(TAG, "Playing 'num' animation in main loop...");
            animation_manager_ptr->PlayAnimation("num", SCREEN_BOTH);
        }
        // vTaskDelay(pdMS_TO_TICKS(5000));
        // if (animation_manager_ptr) {
        //     ESP_LOGI(TAG, "Playing 'happy' animation in main loop...");
        //     animation_manager_ptr->PlayAnimation("happy", SCREEN_BOTH);
        // }
    }
}