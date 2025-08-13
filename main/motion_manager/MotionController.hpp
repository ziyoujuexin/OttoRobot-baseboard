#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/servo.hpp"
#include "config.h"
#include "motion_manager/Motion_types.hpp"
#include "motion_manager/ActionManager.hpp"
#include <memory>
#include <string>
#include <map>
#include <atomic>

class MotionController {
public:
    explicit MotionController(Servo& servo_driver, ActionManager& action_manager);
    ~MotionController();
    void init();
    bool queue_command(const motion_command_t& cmd);
    void servo_test(uint8_t channel, uint8_t angle);

private:
    Servo& m_servo_driver; 
    ActionManager& m_action_manager;
    QueueHandle_t m_motion_queue; 
    std::map<uint8_t, std::string> m_gait_command_map; // Map command code to action name

    // Mapping from logical joint to physical servo channel
    uint8_t m_joint_channel_map[GAIT_JOINT_COUNT];

    std::atomic<bool> m_interrupt_flag; // 用于抢占的原子中断标志

    void init_joint_channel_map();

    void motion_engine_task();

    void execute_action(const RegisteredAction& action);
    void execute_gait(const RegisteredAction& action);
    
    void home();

    static void start_task_wrapper(void* _this) {
        static_cast<MotionController*>(_this)->motion_engine_task();
    }
};
