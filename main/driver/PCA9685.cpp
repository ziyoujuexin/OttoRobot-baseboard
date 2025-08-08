#include "PCA9685.hpp"
#include <esp_log.h>
#include <i2cdev.h>

static const char *TAG = "PCA9685";

PCA9685::PCA9685() {
    memset(&dev, 0, sizeof(dev));
    dev.cfg.scl_pullup_en = 1;
    dev.cfg.sda_pullup_en = 1;
    dev.cfg.master.clk_speed = 40000; // 一个小坑，在这里修改无效，见pca9685_init_desc中的修改。
}

void PCA9685::init() {
    // Initialize the I2C device descriptor
    ESP_LOGI(TAG, "Initializing PCA9685 descriptor");
    pca9685_init_desc(&dev, PCA9685_I2C_ADDR, (i2c_port_t)I2C_PORT, (gpio_num_t)SDA_PIN, (gpio_num_t)SCL_PIN);

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

void PCA9685::set_angle(uint8_t channel, uint8_t angle) {
    if (angle > 180) {
        ESP_LOGW(TAG, "Angle %d is out of range (0-180). Clamping to 180.", angle);
        angle = 180;
    }

    uint16_t pwm_value = map_angle_to_pwm(angle);
    ESP_LOGD(TAG, "Setting channel %d to angle %d (PWM: %d)", channel, angle, pwm_value);

    esp_err_t err = pca9685_set_pwm_value(&dev, channel, pwm_value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM value for channel %d: %s", channel, esp_err_to_name(err));
    }
}

void PCA9685::home_all() {
    ESP_LOGI(TAG, "Homing all servos to 90 degrees.");
    for (uint8_t i = 0; i < 16; ++i) {
        set_angle(i, 90);
    }
}

uint16_t PCA9685::map_angle_to_pwm(uint8_t angle) {
    // Standard servo range is 500-2500us for 0-180 degrees.
    // PCA9685 resolution is 12-bit (4096 steps).
    // PWM frequency is 50Hz, so each step is (1/50)/4096 = 4.88us.
    const uint16_t min_pulse_us = 500;
    const uint16_t max_pulse_us = 2500;
    const float us_per_step = 1000000.0f / (PWM_FREQ_HZ * 4096.0f);

    uint32_t pulse_us = min_pulse_us + (uint32_t)((max_pulse_us - min_pulse_us) * (angle / 180.0f));
    return (uint16_t)(pulse_us / us_per_step);
}
