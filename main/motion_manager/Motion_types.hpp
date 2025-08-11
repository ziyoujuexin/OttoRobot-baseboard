#pragma once

#include <cstdint>
#include "config.h"
#include "nvs.h"

#define MOTION_NAME_MAX_LEN NVS_KEY_NAME_MAX_SIZE
#define MAX_ACTIONS_PER_GROUP 10
#pragma once

#include <cstdint>

#define MOTION_NAME_MAX_LEN NVS_KEY_NAME_MAX_SIZE
#define MAX_ACTIONS_PER_GROUP 10

const int GAIT_JOINT_COUNT = static_cast<int>(ServoChannel::SERVO_COUNT);
typedef struct {
    float amplitude[GAIT_JOINT_COUNT];   // Amplitude of oscillation
    float offset[GAIT_JOINT_COUNT];      // Center point of oscillation
    float phase_diff[GAIT_JOINT_COUNT];  // Phase difference relative to the cycle
} motion_params_t;


// 定义动作的类型
enum class ActionType : uint8_t {
    GAIT_PERIODIC,  // 基于gait函数的周期性动作
    // KEYFRAME_SEQUENCE // 为未来扩展预留：简单的关键帧动作
};

// 定义一个注册在系统中的独立动作
typedef struct {
    char name[MOTION_NAME_MAX_LEN]; // 动作名称 (将作为NVS的key)
    ActionType type;                // 动作类型
    bool is_atomic;                 // 是否为原子操作，执行时不可中断
    uint32_t default_steps;         // 执行该动作的默认步数
    uint32_t default_speed_ms;      // 执行该动作的默认速度 (周期)
    motion_params_t params;         // 动作参数
} RegisteredAction;

// 定义动作组的执行模式
enum class ExecutionMode : uint8_t {
    SEQUENTIAL,     // 组内动作按顺序执行
    SIMULTANEOUS    // 组内动作同时执行 (由Mixer处理)
};

// 定义一个注册在系统中的动作组
typedef struct {
    char name[MOTION_NAME_MAX_LEN]; // 组名称
    ExecutionMode mode;             // 执行模式
    uint8_t action_count;           // 组内动作数量
    char action_names[MAX_ACTIONS_PER_GROUP][MOTION_NAME_MAX_LEN]; // 组内包含的动作名称列表
} RegisteredGroup;
