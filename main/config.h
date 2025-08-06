#pragma once

#include "driver/uart.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "freertos/semphr.h"

typedef struct {
    uint8_t motion_type; // 具体指令, e.g., MOVE_RIGHT, ARM_SWING
    uint8_t param;       // 指令的参数, e.g., 角度, 速度
} motion_command_t;

// 舵机数量定义 - 扩展为8个舵机
#define SERVO_COUNT 8

// 运动状态定义
#define MOTION_STOP        0x00
#define MOTION_FORWARD     0x01
#define MOTION_BACKWARD    0x02
#define MOTION_LEFT        0x03
#define MOTION_RIGHT       0x04
#define MOTION_WAVE_HAND   0x05
#define MOTION_MOVE_EAR    0x06

extern QueueHandle_t motion_queue;
