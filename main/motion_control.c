#include "motion_control.h"
#include "esp32c3_szp.h"
#include <math.h>
#define PI 3.1415926

/**
 * @brief 通用步态函数，移植自xiaozhi工程
 * @param steps 步数
 * @param period_ms 周期时间(毫秒)
 * @param params 运动参数
 */
void motion_gait(int steps, int period_ms, const motion_params_t* params) {
    int total_frames = steps * 20;  // 每步20帧
    for (int frame = 0; frame < total_frames; frame++) {
        float t = (float)frame / 20;  // 时间参数
        for (int i = 0; i < SERVO_COUNT; i++) {
            // 计算舵机角度：基准90度 + 偏移 + 振幅*正弦波
            int angle = 90 + params->offset[i] + params->amplitude[i] * sin(2 * PI * t + params->phase_diff[i]);
            
            // 限制角度范围在0-180度
            if (angle < 0) angle = 0;
            if (angle > 180) angle = 180;
            
            set_servo_angle(i, angle);  // 使用新的直接控制函数
        }
        vTaskDelay(period_ms / 20 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief 将所有舵机设置到中心位置
 */
void motion_home(void) {
    ESP_LOGI("Motion", "设置所有舵机到中心位置");
    for (int i = 0; i < SERVO_COUNT; i++) {
        set_servo_angle(i, 90);  // 使用新的直接控制函数
    }
}

/**
 * @brief 行走动作
 * @param steps 步数
 * @param speed 速度(毫秒)
 * @param direction 方向(1:前进, -1:后退)
 */
void motion_walk(int steps, int speed, int direction) {
    ESP_LOGI("Motion", "执行行走动作: %d步, 速度%dms, 方向%d", steps, speed, direction);
    
    // 参考xiaozhi工程参数
    motion_params_t walk_params = {
        .amplitude = {30 * direction, 30 * direction, 30, 30},
        .offset = {0, 0, 5, -5},
        .phase_diff = {0, PI, -PI/2, -PI/2}
    };
    motion_gait(steps, speed, &walk_params);
}

/**
 * @brief 转向动作
 * @param steps 步数
 * @param speed 速度(毫秒)
 * @param direction 方向(1:左转, -1:右转)
 */
void motion_turn(int steps, int speed, int direction) {
    ESP_LOGI("Motion", "执行转向动作: %d步, 速度%dms, 方向%d", steps, speed, direction);
    
    // 转向时左右腿幅度不同
    motion_params_t turn_params = {
        .amplitude = {30 * direction, 10 * direction, 30, 30},
        .offset = {0, 0, 5, -5},
        .phase_diff = {0, PI, -PI/2, -PI/2}
    };
    motion_gait(steps, speed, &turn_params);
}

/**
 * @brief 跳跃动作
 * @param steps 跳跃次数
 * @param speed 速度(毫秒)
 */
void motion_jump(int steps, int speed) {
    ESP_LOGI("Motion", "执行跳跃动作: %d次, 速度%dms", steps, speed);
    
    int start[4] = {90, 90, 90, 90};      // 起始位置
    int crouch[4] = {60, 60, 120, 120};   // 蹲下位置
    
    for (int s = 0; s < steps; s++) {
        // 蹲下
        for (int i = 0; i < 4; i++) {
            set_servo_angle(i, crouch[i]);
        }
        vTaskDelay(speed / portTICK_PERIOD_MS);
        
        // 跳起
        for (int i = 0; i < 4; i++) {
            set_servo_angle(i, start[i]);
        }
        vTaskDelay(speed / portTICK_PERIOD_MS);
    }
}

/**
 * @brief 挥手动作
 */
void motion_wave_hand(void) {
    ESP_LOGI("Motion", "执行挥手动作");
    
    // 挥手动作：左手臂舵机4和5协调运动
    for (int wave = 0; wave < 3; wave++) {
        // 抬起手臂
        set_servo_angle(4, 45);   // 左手臂舵机1
        set_servo_angle(5, 135);  // 左手臂舵机2
        vTaskDelay(500 / portTICK_PERIOD_MS);
        
        // 放下手臂
        set_servo_angle(4, 90);   // 回到中心位置
        set_servo_angle(5, 90);   // 回到中心位置
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief 动耳朵动作
 */
void motion_move_ear(void) {
    ESP_LOGI("Motion", "执行动耳朵动作");
    
    // 动耳朵动作：左耳朵舵机6和7协调运动
    for (int move = 0; move < 5; move++) {
        // 耳朵向前
        set_servo_angle(6, 60);   // 左耳朵舵机1
        set_servo_angle(7, 120);  // 左耳朵舵机2
        vTaskDelay(300 / portTICK_PERIOD_MS);
        
        // 耳朵向后
        set_servo_angle(6, 120);  // 左耳朵舵机1
        set_servo_angle(7, 60);   // 左耳朵舵机2
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
    
    // 回到中心位置
    set_servo_angle(6, 90);
    set_servo_angle(7, 90);
}
