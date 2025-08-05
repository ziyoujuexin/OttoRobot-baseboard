#include "servo.hpp"
#include "iot_servo.h"
#include "esp_log.h"
#include "config.h"

static const char *SERVO_TAG = "SERVO";

Servo::Servo() {}

void Servo::init() {
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                GPIO_NUM_0, GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3
            },
            .ch = {
                LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3
            },
        },
        .channel_number = SERVO_COUNT,
    };
    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
    ESP_LOGI(SERVO_TAG, "Servo driver initialized using espressif/servo component.");
}

void Servo::set_angle(uint8_t channel, uint8_t angle) {
    if (channel >= SERVO_COUNT) {
        ESP_LOGE(SERVO_TAG, "Invalid servo channel: %d", channel);
        return;
    }
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, channel, angle);
}

void Servo::home_all() {
    ESP_LOGI(SERVO_TAG, "Setting all servos to home position (90 degrees).");
    for (int i = 0; i < SERVO_COUNT; i++) {
        set_angle(i, 90);
    }
}
