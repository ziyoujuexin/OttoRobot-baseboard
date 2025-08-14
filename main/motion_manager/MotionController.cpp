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
      m_actions_mutex(NULL),
      m_interrupt_flag(false) {}

MotionController::~MotionController() {
    if (m_motion_queue != NULL) {
        vQueueDelete(m_motion_queue);
    }
    if (m_actions_mutex != NULL) {
        vSemaphoreDelete(m_actions_mutex);
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

    m_actions_mutex = xSemaphoreCreateMutex();
    if (m_actions_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create actions mutex");
        return;
    }

    xTaskCreate(start_task_wrapper, "motion_engine_task", 4096, this, 5, NULL);
    xTaskCreate(start_mixer_task_wrapper, "motion_mixer_task", 4096, this, 6, NULL); // Higher priority for mixer
    ESP_LOGI(TAG, "Motion Controller initialized and tasks started.");
}

bool MotionController::queue_command(const motion_command_t& cmd) {
    if (cmd.motion_type == MOTION_STOP) {
        ESP_LOGI(TAG, "Interrupt flag set by STOP command.");
        m_interrupt_flag.store(true);
        // Also send to queue to trigger stop logic
    }

    if (xQueueSend(m_motion_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Motion queue is full. Command dropped.");
        return false;
    }
    return true;
}

// --- Core Engine Task (Dispatcher) ---
void MotionController::motion_engine_task() {
    ESP_LOGI(TAG, "Motion engine (dispatcher) task running...");
    motion_command_t received_cmd;
    while (1) {
        if (xQueueReceive(m_motion_queue, &received_cmd, portMAX_DELAY)) {
            
            // Global STOP command logic
            if (m_interrupt_flag.load() && received_cmd.motion_type == MOTION_STOP) {
                ESP_LOGW(TAG, "STOP command received. Clearing all actions and queue.");
                if (xSemaphoreTake(m_actions_mutex, portMAX_DELAY) == pdTRUE) {
                    m_active_actions.clear();
                    xSemaphoreGive(m_actions_mutex);
                }
                xQueueReset(m_motion_queue); // Clear pending commands
                m_interrupt_flag.store(false);
                home();
                continue;
            }

            // Check for atomic action block
            if (xSemaphoreTake(m_actions_mutex, portMAX_DELAY) == pdTRUE) {
                bool is_blocked_by_atomic = false;
                for (const auto& action : m_active_actions) {
                    if (action.is_atomic) {
                        ESP_LOGW(TAG, "Ignoring command (0x%02X) because atomic action '%s' is running.", 
                                 received_cmd.motion_type, action.name);
                        is_blocked_by_atomic = true;
                        break;
                    }
                }
                
                if (is_blocked_by_atomic) {
                    xSemaphoreGive(m_actions_mutex);
                    continue; // Ignore new command
                }

                // Add new action to the active list
                if (m_gait_command_map.count(received_cmd.motion_type)) {
                    const std::string& action_name = m_gait_command_map[received_cmd.motion_type];
                    RegisteredAction action_to_add;
                    if (m_action_manager.get_action(action_name, action_to_add)) {
                        // Here you could add logic to replace existing actions of the same type
                        // For now, we just add it to the list.
                        m_active_actions.push_back(action_to_add);
                        ESP_LOGI(TAG, "Action '%s' added to active list.", action_to_add.name);
                    }
                }
                xSemaphoreGive(m_actions_mutex);
            }
        }
    }
}

// --- Motion Mixer Task ---
void MotionController::motion_mixer_task() {
    ESP_LOGI(TAG, "Motion mixer task running...");
    const int control_period_ms = 20; // 50Hz control rate for mixing
    
    // Add a frame counter to each action
    std::map<std::string, int> action_frame_counters;

    while (1) {
        float final_angles[GAIT_JOINT_COUNT];
        for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
            final_angles[i] = -1.0f; // -1 indicates not set
        }

        bool needs_homing = false;

        if (xSemaphoreTake(m_actions_mutex, portMAX_DELAY) == pdTRUE) {
            
            size_t size_before = m_active_actions.size();

            m_active_actions.erase(
                std::remove_if(m_active_actions.begin(), m_active_actions.end(),
                    [&](RegisteredAction& action) {
                        int frames_per_gait = action.gait_period_ms / control_period_ms;
                        int total_frames = action.default_steps * frames_per_gait;
                        if (action_frame_counters[action.name] >= total_frames) {
                            ESP_LOGI(TAG, "Action '%s' finished and removed.", action.name);
                            action_frame_counters.erase(action.name);
                            return true; // Remove this action
                        }
                        return false;
                    }),
                m_active_actions.end()
            );

            size_t size_after = m_active_actions.size();

            if (size_before > 0 && size_after == 0) {
                needs_homing = true;
            }

            for (auto& action : m_active_actions) {
                if (action.gait_period_ms == 0) continue;

                int frames_per_gait = action.gait_period_ms / control_period_ms;
                if (frames_per_gait == 0) continue;

                int current_frame = action_frame_counters[action.name];
                float t = (float)(current_frame % frames_per_gait) / frames_per_gait;

                for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                    if (final_angles[i] < 0.0f) { // If not already set by a higher priority action
                        float amp = action.params.amplitude[i];
                        if (std::abs(amp) > 0.01f) {
                            float offset = action.params.offset[i];
                            float phase = action.params.phase_diff[i];
                            float angle = 90.0f + offset + amp * sin(2 * PI * t + phase);
                            
                            if (angle < 0) angle = 0;
                            if (angle > 180) angle = 180;
                            final_angles[i] = angle;
                        }
                    }
                }
                action_frame_counters[action.name]++;
            }
            xSemaphoreGive(m_actions_mutex);
        }

        // Apply final angles to hardware
        for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
            if (final_angles[i] >= 0.0f) {
                uint8_t channel = m_joint_channel_map[i];
                m_servo_driver.set_angle(channel, static_cast<int>(final_angles[i]));
            }
        }

        if (needs_homing) {
            home();
        }

        vTaskDelay(pdMS_TO_TICKS(control_period_ms));
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

