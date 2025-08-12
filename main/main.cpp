#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/PCA9685.hpp"
#include "MotionController.hpp"
#include "UartHandler.hpp"
#include "web_server/WebServer.hpp"
#include <memory>
#include <i2cdev.h>

static const char *TAG = "MAIN";

// create all handlers statically
PCA9685 servo_driver;
auto motion_controller_ptr = std::make_unique<MotionController>(servo_driver);
auto uart_handler_ptr = std::make_unique<UartHandler>(*motion_controller_ptr);
auto web_server_ptr = std::make_unique<WebServer>(*motion_controller_ptr);

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Application startup.");
    
    // Initialize I2C bus for peripherals
    i2cdev_init();

    // Create the servo driver
    servo_driver.init();

    // Create and initialize the motion controller
    motion_controller_ptr = std::make_unique<MotionController>(servo_driver);
    motion_controller_ptr->init();

    // Create and initialize the UART command handler
    uart_handler_ptr = std::make_unique<UartHandler>(*motion_controller_ptr);
    uart_handler_ptr->init();
    
    // Create and start the web server for remote control
    web_server_ptr = std::make_unique<WebServer>(*motion_controller_ptr);
    web_server_ptr->start();

    ESP_LOGI(TAG, "Initialization complete. All modules started.");

    // Clean and re-register default actions for debugging
    // motion_controller_ptr->delete_action_from_nvs("walk_forward");
    // motion_controller_ptr->delete_action_from_nvs("walk_backward");
    // motion_controller_ptr->delete_action_from_nvs("turn_left");
    // motion_controller_ptr->delete_action_from_nvs("turn_right");
    // motion_controller_ptr->register_default_actions();

    while (true) {
        motion_controller_ptr->servo_test(15, 90); // 15通道总是用来校准舵机，方便机械安装
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}