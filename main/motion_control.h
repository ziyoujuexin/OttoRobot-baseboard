#define LEFT_LEG     0
#define RIGHT_LEG    1
#define LEFT_FOOT    2
#define RIGHT_FOOT   3
#define LEFT_ARM_1   4  // 新增：左手臂舵机1
#define LEFT_ARM_2   5  // 新增：左手臂舵机2
#define LEFT_EAR_1   6  // 新增：左耳朵舵机1
#define LEFT_EAR_2   7  // 新增：左耳朵舵机2
#define SERVO_COUNT  8  // 更新舵机总数

#pragma once

#include <math.h>

// 步态参数结构体
typedef struct {
    int amplitude[SERVO_COUNT];   // 振幅
    int offset[SERVO_COUNT];      // 偏移
    double phase_diff[SERVO_COUNT]; // 相位差
} motion_params_t;

void motion_home(void);
void motion_walk(int steps, int speed, int direction);
void motion_turn(int steps, int speed, int direction);
void motion_jump(int steps, int speed);

// 通用步态函数（移植自xiaozhi）
void motion_gait(int steps, int period_ms, const motion_params_t* params);

// 新增函数声明
void motion_wave_hand(void);
void motion_move_ear(void);