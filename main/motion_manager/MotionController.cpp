#include "MotionController.hpp"
#include "esp_log.h"
#include <cmath>

#define PI 3.1415926
static const char* TAG = "MotionController";

// --- Constructor / Destructor ---
MotionController::MotionController(Servo& servo_driver, ActionManager& action_manager) 
    : m_servo_driver(servo_driver), 
      m_action_manager(action_manager),
      m_motion_queue(NULL),
      m_interrupt_flag(false) {}

MotionController::~MotionController() {
    if (m_motion_queue != NULL) {
        vQueueDelete(m_motion_queue);
    }
}

void MotionController::init_joint_channel_map() {
    for (uint8_t i = 0; i < static_cast<uint8_t>(ServoChannel::SERVO_COUNT); ++i) {
        m_joint_channel_map[i] = i;
    }
    ESP_LOGI(TAG, "Joint-to-Channel map identity-initialized for %d channels.", static_cast<int>(ServoChannel::SERVO_COUNT));
}

// --- Public Methods ---
void MotionController::init() {
    init_joint_channel_map();

    m_gait_command_map[MOTION_FORWARD] = "walk_forward";
    m_gait_command_map[MOTION_BACKWARD] = "walk_backward";
    m_gait_command_map[MOTION_LEFT] = "turn_left";
    m_gait_command_map[MOTION_RIGHT] = "turn_right";
    m_gait_command_map[MOTION_WAVE_HAND] = "wave_hand";
    m_gait_command_map[MOTION_MOVE_EAR] = "wiggle_ears";
    m_gait_command_map[MOTION_NOD_HEAD] = "nod_head";
    m_gait_command_map[MOTION_SHAKE_HEAD] = "shake_head";
    m_gait_command_map[MOTION_SINGLE_LEG] = "single_leg";

    m_motion_queue = xQueueCreate(10, sizeof(motion_command_t));
    if (m_motion_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create motion queue");
        return;
    }

    xTaskCreate(start_task_wrapper, "motion_engine_task", 4096, this, 5, NULL);
    ESP_LOGI(TAG, "Motion Controller initialized and task started.");
}

bool MotionController::queue_command(const motion_command_t& cmd) {
    if (cmd.motion_type == MOTION_STOP) {
        ESP_LOGI(TAG, "Interrupt flag set by STOP command.");
        m_interrupt_flag.store(true);
    }

    if (xQueueSend(m_motion_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Motion queue is full. Command dropped.");
        return false;
    }
    return true;
}

// --- Core Engine Task ---
void MotionController::motion_engine_task() {
    ESP_LOGI(TAG, "Motion engine task running...");
    motion_command_t received_cmd;
    while (1) {
        if (xQueueReceive(m_motion_queue, &received_cmd, portMAX_DELAY)) {
            m_interrupt_flag.store(false);
            ESP_LOGI(TAG, "Executing motion command type: 0x%02X", received_cmd.motion_type);

            if (m_gait_command_map.count(received_cmd.motion_type)) {
                const std::string& action_name = m_gait_command_map[received_cmd.motion_type];
                RegisteredAction action_to_run;
                if (m_action_manager.get_action(action_name, action_to_run)) {
                    execute_action(action_to_run);
                }
                continue; 
            }

            switch (received_cmd.motion_type) {
                case MOTION_STOP:
                    home();
                    break;
                case MOTION_RUN_DEMO_GROUP:
                    ESP_LOGW(TAG, "Demo group execution is not yet refactored.");
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown motion type: 0x%02X", received_cmd.motion_type);
                    break;
            }
        }
    }
}

// --- Action Execution ---
void MotionController::execute_action(const RegisteredAction& action) {
    // The action details are now printed by ActionManager when properties are updated.
    if (action.type == ActionType::GAIT_PERIODIC) {
        execute_gait(action);
    } 
}

void MotionController::execute_gait(const RegisteredAction& action) {
    const int control_period_ms = 10; // 100Hz control rate
    if (action.gait_period_ms == 0) {
        ESP_LOGE(TAG, "Gait period cannot be zero.");
        return;
    }

    int frames_per_gait = action.gait_period_ms / control_period_ms;
    if (frames_per_gait == 0) {
        ESP_LOGE(TAG, "Gait period is too short for the control rate.");
        return;
    }
    int total_frames = action.default_steps * frames_per_gait;

    ESP_LOGI(TAG, "Executing gait '%s': %d steps, %dms/step, %d total frames", 
             action.name, (int)action.default_steps, (int)action.gait_period_ms, total_frames);

    for (int frame = 0; frame < total_frames; frame++) {
        if (m_interrupt_flag.load()) {
            ESP_LOGW(TAG, "Gait interrupted.");
            break;
        }

        float t = (float)frame / frames_per_gait;

        for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
            float amp = action.params.amplitude[i];
            if (std::abs(amp) < 0.01f) continue;
            float offset = action.params.offset[i];
            float phase = action.params.phase_diff[i];
            float angle = 90.0f + offset + amp * sin(2 * PI * t + phase);
            
            if (angle < 0) angle = 0;
            if (angle > 180) angle = 180;

            uint8_t channel = m_joint_channel_map[i];
            m_servo_driver.set_angle(channel, static_cast<int>(angle));
        }
        vTaskDelay(pdMS_TO_TICKS(control_period_ms));
    }

    if (!m_interrupt_flag.load()) {
        home();
    }
}

void MotionController::home() {
    ESP_LOGI(TAG, "Homing all servos to 90 degrees.");
    m_servo_driver.home_all();
    vTaskDelay(pdMS_TO_TICKS(100));
}

void MotionController::servo_test(uint8_t channel, uint8_t angle) {
    m_servo_driver.set_angle(channel, angle);
}

