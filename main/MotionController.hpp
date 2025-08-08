#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/servo.hpp"
#include "config.h"
#include <memory>

// 动作控制命令结构体


// 步态参数结构体
typedef struct {
    int amplitude[SERVO_COUNT];
    int offset[SERVO_COUNT];
    double phase_diff[SERVO_COUNT];
} motion_params_t;

class MotionController {
public:
    // 构造函数公开，并接收引用
    explicit MotionController(Servo& servo_driver);
    ~MotionController();
    void init();
    bool queue_command(const motion_command_t& cmd);
    void servo_test(uint8_t channel, uint8_t angle);

private:
    Servo& m_servo_driver; 
    
    QueueHandle_t m_motion_queue; 

    // 运行在独立任务中的主循环
    void motion_task_handler();

    // 将原C函数改为类的私有方法
    void home();
    void gait(int steps, int period_ms, const motion_params_t* params);
    void walk(int steps, int speed, int direction);
    void turn(int steps, int speed, int direction);
    void wave_hand();
    void move_ear();

    // 静态的Task启动函数，用于适配FreeRTOS的xTaskCreate
    static void start_task_wrapper(void* _this) {
        static_cast<MotionController*>(_this)->motion_task_handler();
    }
};