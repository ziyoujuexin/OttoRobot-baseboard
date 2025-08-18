#pragma once

#include <cstdint>
#include <vector>
#include "config.h"
#include "nvs.h"

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
    uint32_t gait_period_ms;        // 单个步态周期的默认时长 (ms)
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


// 定义一个运动指令
typedef struct {
    uint8_t motion_type; // 运动类型 (例如：前进、后退、停止)
    std::vector<uint8_t> params;       // 可变长度的参数
} motion_command_t;

// 定义面部位置结构体
// #pragma pack(1)
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    bool detected;
} FaceLocation;

// 定义步态函数指针类型
typedef float (*gait_function_t)(float);

// 定义一个步态
typedef struct {
    gait_function_t function; // 步态函数
    const char* name;         // 步态名称
} Gait;

// 定义一个动作实例
typedef struct {
    RegisteredAction action; // 动作定义
    uint32_t remaining_steps; // 剩余步数
    uint32_t start_time_ms;   // 开始时间
} ActionInstance;