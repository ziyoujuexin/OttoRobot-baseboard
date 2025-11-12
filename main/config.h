#pragma once

// I2C configuration



#include "driver/uart.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "freertos/semphr.h"

// Wi-Fi Configuration
#define WIFI_SSID     "LIANQIU-2"
#define WIFI_PASSWORD "lianqiu123"

#define MOTION_STOP           0x00
#define MOTION_FORWARD        0x01
#define MOTION_BACKWARD       0x02
#define MOTION_LEFT           0x03
#define MOTION_RIGHT          0x04

#define MOTION_WAVE_HAND      0x05
#define MOTION_MOVE_EAR       0x06
#define MOTION_NOD_HEAD       0x07
#define MOTION_SHAKE_HEAD     0x08
#define MOTION_WALK_BACKWARD_KF 0x09
#define MOTION_WAVE_HELLO     0x0C 
#define MOTION_FACE_TRACE     0x0A
#define MOTION_FACE_END       0x0B

#define MOTION_HAPPY          0x10 
#define MOTION_LOOKAROUND     0x11
#define MOTION_DANCE          0x12
#define MOTION_FUNNY          0x13
#define MOTION_VERY_HAPPY     0x14
#define MOTION_ANGRY          0x15
#define MOTION_CRYING         0x16
#define MOTION_SURPRISED      0x17
#define MOTION_SAD            0x18
#define MOTION_LOVOT_SHAKE    0x19
#define MOTION_WAKE_DETECT    0xC0
#define MOTION_PLAY_ANIMATION 0xD0
#define MOTION_PLAY_MOTION    0xD1

#define MOTION_TRACKING_L     0x1A
#define MOTION_TRACKING_R     0x1B
#define MOTION_WALK_FORWARD_KF 0x1C
#define MOTION_STARTLE_AND_SIGH 0x1D


#define MOTION_TUNE_PARAM     0x20
#define MOTION_SAVE_PARAMS    0x21
#define MOTION_GET_PARAMS     0x22
#define MOTION_WAKE_DETECT    0xC0
#define MOTION_SOUND_SOURCE   0xD2
#define MOTION_SERVO_CONTROL  0xF0

enum class ServoChannel : uint8_t {
    LEFT_EAR_LIFT = 0,
    LEFT_EAR_SWING = 1,
    RIGHT_EAR_LIFT = 2,
    RIGHT_EAR_SWING = 3,
    HEAD_TILT = 4,
    HEAD_PAN = 5,
    RIGHT_ARM_SWING = 6,
    LEFT_ARM_LIFT = 7,
    LEFT_ARM_SWING = 8,
    RIGHT_ARM_LIFT = 9,
    LEFT_LEG_ROTATE = 10,
    LEFT_ANKLE_LIFT = 11,
    RIGHT_LEG_ROTATE = 12,
    RIGHT_ANKLE_LIFT = 13,

    SERVO_COUNT // This automatically gives the number of servos
};
extern QueueHandle_t motion_queue;
