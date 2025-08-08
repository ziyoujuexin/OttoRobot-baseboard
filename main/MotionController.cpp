#include "MotionController.hpp"
#include "esp_log.h"
#include <cmath>

#define PI 3.1415926
static const char* TAG = "MotionController";

// 来自 motion_control.h 的舵机定义
#define LEFT_LEG     0
#define RIGHT_LEG    1
#define LEFT_FOOT    2
#define RIGHT_FOOT   3
#define LEFT_ARM_1   4
#define LEFT_ARM_2   5
#define LEFT_EAR_1   6
#define LEFT_EAR_2   7

MotionController::MotionController(Servo& servo_driver) : m_servo_driver(servo_driver), m_motion_queue(NULL) {}

MotionController::~MotionController() {
    if (m_motion_queue != NULL) {
        vQueueDelete(m_motion_queue);
    }
}

void MotionController::init() {
    m_motion_queue = xQueueCreate(10, sizeof(motion_command_t));
    if (m_motion_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create motion queue");
        return;
    }
    
    xTaskCreate(start_task_wrapper, "motion_task", 4096, this, 5, NULL);
    ESP_LOGI(TAG, "Motion Controller initialized and task started.");
}

bool MotionController::queue_command(const motion_command_t& cmd) {
    if (xQueueSend(m_motion_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Motion queue is full. Command dropped.");
        return false;
    }
    ESP_LOGI(TAG, "Motion command queued.");
    return true;
}

void MotionController::motion_task_handler() {
    ESP_LOGI(TAG, "Motion task running...");
    motion_command_t received_cmd;

    while (1) {
        // 等待队列中的运动指令
        if (xQueueReceive(m_motion_queue, &received_cmd, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Executing motion command: %d", received_cmd.motion_type);
            switch (received_cmd.motion_type) {
                case MOTION_STOP:
                    home();
                    break;
                case MOTION_FORWARD:
                    walk(2, 500, 1); // 示例参数：2步，500ms周期，前进
                    break;
                case MOTION_BACKWARD:
                    walk(2, 500, -1); // 示例参数：2步，500ms周期，后退
                    break;
                case MOTION_LEFT:
                    turn(2, 500, 1); // 示例参数：2步，500ms周期，左转
                    break;
                case MOTION_RIGHT:
                    turn(2, 500, -1); // 示例参数：2步，500ms周期，右转
                    break;
                case MOTION_WAVE_HAND:
                    wave_hand();
                    break;
                case MOTION_MOVE_EAR:
                    move_ear();
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown motion type: %d", received_cmd.motion_type);
                    break;
            }
             // 动作完成后可以回到home位置，或者等待下一个指令
            // home();

        }
    }
}

// --- 以下是将原C函数移植为类的私有方法 ---

void MotionController::gait(int steps, int period_ms, const motion_params_t* params) {
    int total_frames = steps * 20;
    for (int frame = 0; frame < total_frames; frame++) {
        float t = (float)frame / 20;
        for (int i = 0; i < SERVO_COUNT; i++) {
            int angle = 90 + params->offset[i] + params->amplitude[i] * sin(2 * PI * t + params->phase_diff[i]);
            if (angle < 0) angle = 0;
            if (angle > 180) angle = 180;
            // 注意：这里调用的是m_servo_driver的方法
            m_servo_driver.set_angle(i, angle);
        }
        vTaskDelay(pdMS_TO_TICKS(period_ms / 20));
    }
}

void MotionController::home() {
    ESP_LOGI(TAG, "All servos to home position.");
    // 注意：这里调用的是m_servo_driver的方法
    m_servo_driver.home_all();
}

void MotionController::walk(int steps, int speed, int direction) {
    ESP_LOGI(TAG, "Executing walk: %d steps, speed %dms, direction %d", steps, speed, direction);
    motion_params_t walk_params = {
        .amplitude = {30 * direction, 30 * direction, 30, 30, 0, 0, 0, 0},
        .offset = {0, 0, 5, -5, 0, 0, 0, 0},
        .phase_diff = {0, PI, -PI / 2, -PI / 2, 0, 0, 0, 0}
    };
    gait(steps, speed, &walk_params);
}

void MotionController::turn(int steps, int speed, int direction) {
     ESP_LOGI(TAG, "Executing turn: %d steps, speed %dms, direction %d", steps, speed, direction);
    motion_params_t turn_params = {
        .amplitude = {30 * direction, 10 * direction, 30, 30, 0, 0, 0, 0},
        .offset = {0, 0, 5, -5, 0, 0, 0, 0},
        .phase_diff = {0, PI, -PI / 2, -PI / 2, 0, 0, 0, 0}
    };
    gait(steps, speed, &turn_params);
}

void MotionController::wave_hand() {
    ESP_LOGI(TAG, "Executing wave hand");
    for (int wave = 0; wave < 3; wave++) {
        m_servo_driver.set_angle(LEFT_ARM_1, 45);
        m_servo_driver.set_angle(LEFT_ARM_2, 135);
        vTaskDelay(pdMS_TO_TICKS(500));
        m_servo_driver.set_angle(LEFT_ARM_1, 90);
        m_servo_driver.set_angle(LEFT_ARM_2, 90);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void MotionController::move_ear() {
    ESP_LOGI(TAG, "Executing move ear");
    for (int move = 0; move < 5; move++) {
        m_servo_driver.set_angle(LEFT_EAR_1, 60);
        m_servo_driver.set_angle(LEFT_EAR_2, 120);
        vTaskDelay(pdMS_TO_TICKS(300));
        m_servo_driver.set_angle(LEFT_EAR_1, 120);
        m_servo_driver.set_angle(LEFT_EAR_2, 60);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    m_servo_driver.set_angle(LEFT_EAR_1, 90);
    m_servo_driver.set_angle(LEFT_EAR_2, 90);
}

void MotionController::servo_test(uint8_t channel, uint8_t angle) {
    // 注意：这里调用的是m_servo_driver的方法
    ESP_LOGI(TAG, "Executing servo test: channel %d, angle %d", channel, angle);
    m_servo_driver.set_angle(channel, angle);
}
