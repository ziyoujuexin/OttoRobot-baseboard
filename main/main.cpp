#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/PCA9685.hpp"
#include "MotionController.hpp"
#include "UartHandler.hpp"
#include <memory>
#include <i2cdev.h>

static const char *TAG = "MAIN";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Application startup.");

    // Initialize I2C bus
    i2cdev_init();

    PCA9685 servo_driver;
    servo_driver.init();

    auto motion_controller_ptr = std::make_unique<MotionController>(servo_driver);
    motion_controller_ptr->init();

    auto uart_handler_ptr = std::make_unique<UartHandler>(*motion_controller_ptr);
    uart_handler_ptr->init();
    
    ESP_LOGI(TAG, "Initialization complete. All modules started.");

    // 保持主任务运行，以确保 unique_ptr 不会被销毁
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}