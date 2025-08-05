#pragma once
#include "driver/uart.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// 舵机数量定义 - 扩展为8个舵机
#define SERVO_COUNT 8

// 运动状态定义
#define MOTION_STOP        0x00
#define MOTION_FORWARD     0x01
#define MOTION_BACKWARD    0x02
#define MOTION_LEFT        0x03
#define MOTION_RIGHT       0x04
#define MOTION_WAVE_HAND   0x05  // 新增：挥手动作
#define MOTION_MOVE_EAR    0x06  // 新增：动耳朵动作

// 运动命令结构体（只定义一次）
typedef struct {
    uint8_t motion_type;
} motion_command_t;

// 舵机控制函数声明
void servo_pwm_init(void);
void set_servo_angle(uint8_t channel, uint8_t angle);
void servo_home_position(void);

// 串口相关函数声明
void uart_init(void);
void uart_receive_task(void *pvParameters);

// 全局队列声明（只声明，不定义）
extern QueueHandle_t motion_queue;

/*********************    直接舵机控制 ↑   *******************/
/***********************************************************/

// 所有I2C、UART、ToF、PCA9548A、PCA9685相关代码已完全移除
// 项目现在只支持直接舵机控制功能

