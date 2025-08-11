#pragma once

#include "driver/uart.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "freertos/semphr.h"

// Wi-Fi Configuration
#define WIFI_SSID     "LIANQIU-2"
#define WIFI_PASSWORD "lianqiu123"

typedef struct {
    uint8_t motion_type; // 具体指令, e.g., MOVE_RIGHT, ARM_SWING
    uint8_t param;       // 指令的参数, e.g., 角度, 速度
} motion_command_t;

// 舵机数量定义 - 扩展为8个舵机

// 运动状态定义
#define MOTION_STOP        0x00
#define MOTION_FORWARD     0x01
#define MOTION_BACKWARD    0x02
#define MOTION_LEFT        0x03
#define MOTION_RIGHT       0x04
#define MOTION_WAVE_HAND   0x05
#define MOTION_MOVE_EAR    0x06

// Custom groups, hardcoded for UART compatibility
#define MOTION_RUN_DEMO_GROUP 0x10

enum class ServoChannel : uint8_t {
    // EAR
    LEFT_EAR_LIFT = 0,
    RIGHT_EAR_LIFT = 2,

    LEFT_EAR_SWING = 1,
    RIGHT_EAR_SWING = 3,
    // HEAD
    HEAD_PAN = 5,
    HEAD_TILT = 4,
    // ARM
    LEFT_ARM_SWING = 6,
    RIGHT_ARM_SWING = 8,

    LEFT_ARM_LIFT = 7,
    RIGHT_ARM_LIFT = 9,
    // LEG
    LEFT_LEG_ROTATE = 10,
    RIGHT_LEG_ROTATE = 12,
    // Ankle
    LEFT_ANKLE_LIFT = 11,
    RIGHT_ANKLE_LIFT = 13,

    SERVO_COUNT // This automatically gives the number of servos
};
extern QueueHandle_t motion_queue;
