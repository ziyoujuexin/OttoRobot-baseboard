#include "PCA9685.hpp"
#include "motion_manager/ServoCalibration.hpp"
#include <esp_log.h>
#include <cstring>
#include <i2cdev.h>

static const char *TAG = "PCA9685";

PCA9685::PCA9685() {
    memset(&dev, 0, sizeof(dev));
    dev.cfg.scl_pullup_en = 1;
    dev.cfg.sda_pullup_en = 1;
    // dev.cfg.master.clk_speed = 40000; // 一个小坑，在这里修改无效，见pca9685_init_desc中的修改。
}

void PCA9685::init() {
    // Initialize the I2C device descriptor
    ESP_LOGI(TAG, "Initializing PCA9685 descriptor");
    pca9685_init_desc(&dev, PCA9685_I2C_ADDR, (i2c_port_t)I2C_PORT, (gpio_num_t)SDA_PIN, (gpio_num_t)SCL_PIN);
    dev.cfg.master.clk_speed = 40000;
    // Initialize the PCA9685 device
    ESP_LOGI(TAG, "Initializing PCA9685");
    esp_err_t err = pca9685_init(&dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PCA9685: %s", esp_err_to_name(err));
        return;
    }

    // Restart the PCA9685
    ESP_LOGI(TAG, "Restarting PCA9685");
    err = pca9685_restart(&dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart PCA9685: %s", esp_err_to_name(err));
        return;
    }

    // Set the PWM frequency
    ESP_LOGI(TAG, "Setting PWM frequency");
    err = pca9685_set_pwm_frequency(&dev, PWM_FREQ_HZ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM frequency: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "PCA9685 initialized successfully");
}

// TODO: 为了代码的兼容性，这里其实180对应了物理上的120度，后续修改
void PCA9685::set_angle(uint8_t channel, float angle) {
    if (channel > 15) {
        ESP_LOGE(TAG, "Invalid channel: %d. Must be 0-15.", channel);
        return;
    }

    if (channel == 6) {
        angle = 180 - angle;
    } else if (channel == 7) {
        angle = 130 - angle;
    }

    // Get the min and max angles for this specific channel
    float min_angle = ServoCalibration::limits[channel].min;
    float max_angle = ServoCalibration::limits[channel].max;

    // Get the min and max pulse widths for this specific channel
    uint16_t min_pulse_us = ServoCalibration::pulse_limits[channel].min_us;
    uint16_t max_pulse_us = ServoCalibration::pulse_limits[channel].max_us;

    // 将角度(min_angle-max_angle)线性映射到脉冲宽度计数值
    uint16_t pulse = map_angle_to_pwm(angle, min_angle, max_angle, min_pulse_us, max_pulse_us);

    ESP_LOGD(TAG, "Channel: %d, Angle: %.1f, Min: %.1f, Max: %.1f, Pulse: %d", channel, angle, min_angle, max_angle, pulse);

    // 在PCA9685上设置PWM值
    esp_err_t err = pca9685_set_pwm_value(&dev, channel, pulse);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM value for channel %d: %s", channel, esp_err_to_name(err));
    }
}

void PCA9685::home_all() {
    ESP_LOGI(TAG, "Homing all servos to 90 degrees.");
    for (uint8_t i = 0; i < 16; ++i) {
        set_angle(i, 90);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

uint16_t PCA9685::map_angle_to_pwm(float angle, float min_angle, float max_angle, uint16_t min_pulse_us, uint16_t max_pulse_us) {
    // PCA9685 resolution is 12-bit (4096 steps).
    const float us_per_step = 1000000.0f / (PWM_FREQ_HZ * 4096.0f);

    float range_angle = max_angle - min_angle;
    if (range_angle == 0) {
        range_angle = 180.0f; // Prevent division by zero
    }

    // Calculate the percentage of the angle within its allowed range
    float percentage = (angle - min_angle) / range_angle;

    // Apply this percentage to the pulse width range
    uint32_t pulse_us = min_pulse_us + (uint32_t)((max_pulse_us - min_pulse_us) * percentage);

    return (uint16_t)(pulse_us / us_per_step);
}
