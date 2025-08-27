#pragma once

#include "driver/uart.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "freertos/semphr.h"

// Wi-Fi Configuration
#define WIFI_SSID     "LIANQIU-2"
#define WIFI_PASSWORD "lianqiu123"


// 运动状态定义
#define MOTION_STOP        0x00
#define MOTION_FORWARD     0x01
#define MOTION_BACKWARD    0x02
#define MOTION_LEFT        0x03
#define MOTION_RIGHT       0x04
#define MOTION_WAVE_HAND   0x05
#define MOTION_MOVE_EAR    0x06
#define MOTION_NOD_HEAD    0x07
#define MOTION_SHAKE_HEAD  0x08
#define MOTION_SINGLE_LEG  0x09
#define MOTION_FACE_TRACE  0x0A

// Custom commands for face tracking coordination
#define MOTION_TRACKING_L 0x11
#define MOTION_TRACKING_R 0x12

// Custom groups, hardcoded for UART compatibility
#define MOTION_RUN_DEMO_GROUP 0x10

// Commands for Web UI parameter tuning
#define MOTION_TUNE_PARAM    0x20
#define MOTION_SAVE_PARAMS   0x21
#define MOTION_GET_PARAMS    0x22

#define MOTION_WAKE_DETECT   0xC0

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
