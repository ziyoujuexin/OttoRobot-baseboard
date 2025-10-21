#pragma once

#include <cstdint>
#include <vector>
#include "config.h"
#include "nvs.h"

#define MOTION_NAME_MAX_LEN NVS_KEY_NAME_MAX_SIZE
#define MAX_ACTIONS_PER_GROUP 10
#define MAX_KEYFRAMES_PER_ACTION 20 // Maximum number of keyframes in a single action

const int GAIT_JOINT_COUNT = static_cast<int>(ServoChannel::SERVO_COUNT);

// Parameters for GAIT_PERIODIC actions
typedef struct {
    float amplitude[GAIT_JOINT_COUNT];   // Amplitude of oscillation
    float offset[GAIT_JOINT_COUNT];      // Center point of oscillation
    float phase_diff[GAIT_JOINT_COUNT];  // Phase difference relative to the cycle
} motion_params_t;

// Defines the type of an action
enum class ActionType : uint8_t {
    GAIT_PERIODIC,      // Periodic action based on gait functions
    KEYFRAME_SEQUENCE   // Action based on a sequence of keyframes
};

// Defines a single keyframe in a sequence
typedef struct {
    uint16_t transition_time_ms;    // Time to transition to this frame from the previous one (in ms)
    float positions[GAIT_JOINT_COUNT]; // Target positions for each servo at this frame (in degrees)
} Keyframe;

// Holds the data for a keyframe sequence action
typedef struct {
    uint8_t frame_count;
    Keyframe frames[MAX_KEYFRAMES_PER_ACTION];
} KeyframeActionData;

// Defines a registered action in the system
typedef struct {
    char name[MOTION_NAME_MAX_LEN]; // Action name (will be the key in NVS)
    ActionType type;                // The type of action
    bool is_atomic;                 // If true, the action cannot be interrupted
    uint32_t default_steps;         // Default number of times to repeat the action
    
    union {
        // Data for GAIT_PERIODIC actions
        struct {
            uint32_t gait_period_ms;    // Default duration of a single gait cycle (ms)
            motion_params_t params;     // Gait parameters
        } gait;

        // Data for KEYFRAME_SEQUENCE actions
        KeyframeActionData keyframe;
    } data;

} RegisteredAction;

// Defines the execution mode for a group of actions
enum class ExecutionMode : uint8_t {
    SEQUENTIAL,     // Actions in the group are executed sequentially
    SIMULTANEOUS    // Actions in the group are executed simultaneously (handled by the Mixer)
};

// Defines a group of registered actions
typedef struct {
    char name[MOTION_NAME_MAX_LEN]; // Group name
    ExecutionMode mode;             // Execution mode
    uint8_t action_count;           // Number of actions in the group
    char action_names[MAX_ACTIONS_PER_GROUP][MOTION_NAME_MAX_LEN]; // List of action names in the group
} RegisteredGroup;


// Defines a command for the motion controller
typedef struct {
    uint8_t motion_type; // Type of motion (e.g., walk forward, stop)
    std::vector<uint8_t> params;       // Variable length parameters
} motion_command_t;

// Defines the location of a detected face
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    bool detected;
} FaceLocation;

// Defines a pointer to a gait function
typedef float (*gait_function_t)(float);

// Defines a gait
typedef struct {
    gait_function_t function; // Gait function
    const char* name;         // Gait name
} Gait;

// Defines an instance of a running action, holding its state
typedef struct {
    RegisteredAction action;    // The definition of the action
    uint32_t remaining_steps;   // Number of remaining repetitions
    uint32_t start_time_ms;     // Start time of the current step/cycle

    // State for keyframe animations
    uint8_t current_keyframe_index; // Index of the current target keyframe
    uint32_t transition_start_time_ms; // Start time of the transition to the current keyframe
    float start_positions[GAIT_JOINT_COUNT]; // Servo positions at the beginning of the transition

} ActionInstance;