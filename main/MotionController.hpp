#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/servo.hpp"
#include "config.h"
#include "Motion_types.hpp" // 引入新的类型定义
#include "MotionStorage.hpp" // 引入存储类
#include <memory>
#include <string>
#include <map>
#include <atomic>

class MotionController {
public:
    explicit MotionController(Servo& servo_driver);
    ~MotionController();
    void init();
    bool queue_command(const motion_command_t& cmd);
    void servo_test(uint8_t channel, uint8_t angle);

    // --- NVS Storage Interface ---
    bool delete_action_from_nvs(const std::string& action_name);
    bool delete_group_from_nvs(const std::string& group_name);
    std::vector<std::string> list_actions_from_nvs();
    std::vector<std::string> list_groups_from_nvs();

private:
    Servo& m_servo_driver; 
    QueueHandle_t m_motion_queue; 
    std::unique_ptr<MotionStorage> m_storage; // 使用智能指针管理存储实例
    std::map<std::string, RegisteredAction> m_action_cache; // 缓存从NVS加载的动作
    std::map<std::string, RegisteredGroup> m_group_cache; // 缓存从NVS加载的动作组

    // Mapping from logical joint to physical servo channel
    uint8_t m_joint_channel_map[GAIT_JOINT_COUNT];

    std::atomic<bool> m_interrupt_flag; // 用于抢占的原子中断标志

    // 初始化逻辑关节到物理通道的映射
    void init_joint_channel_map();

    // 核心任务循环
    void motion_engine_task();

    // 动作执行器
    void execute_action(const RegisteredAction& action);
    void execute_gait(const RegisteredAction& action);
    
    // 旧的非gait动作，暂时保留以兼容
    void wave_hand();
    void move_ear();
    void home();

    // Helper for debugging
    void print_action_details(const RegisteredAction& action);

    // 初始化默认数据
    void register_default_actions();
    void register_default_groups();

    // 静态的Task启动函数
    static void start_task_wrapper(void* _this) {
        static_cast<MotionController*>(_this)->motion_engine_task();
    }
};
