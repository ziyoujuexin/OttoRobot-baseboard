#include "MotionController.hpp"
#include "esp_log.h"
#include <cmath>
#include <string.h>

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
    if (m_face_data_mutex != NULL) {
        vSemaphoreDelete(m_face_data_mutex);
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

    m_face_data_mutex = xSemaphoreCreateMutex();
    if (m_face_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create face data mutex");
        return;
    }

    // Initialize PID controller variables
    m_last_face_location = {0, 0, 0, 0, 0};
    m_pid_pan_error_last = 0;
    m_pid_tilt_error_last = 0;
    m_pid_pan_integral = 0;
    m_pid_tilt_integral = 0;

    xTaskCreate(start_task_wrapper, "motion_engine_task", 4096, this, 5, NULL);
    xTaskCreate(start_mixer_task_wrapper, "motion_mixer_task", 4096, this, 6, NULL); // Higher priority for mixer
    xTaskCreate(start_face_tracking_task_wrapper, "face_tracking_task", 4096, this, 5, NULL);
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

                // --- Handle different command types ---
                switch (received_cmd.motion_type) {
                    case MOTION_FACE_TRACE: {
                        if (received_cmd.params.size() == sizeof(FaceLocation)) {
                            if (xSemaphoreTake(m_face_data_mutex, portMAX_DELAY) == pdTRUE) {
                                memcpy(&m_last_face_location, received_cmd.params.data(), sizeof(FaceLocation));
                                xSemaphoreGive(m_face_data_mutex);
                                ESP_LOGD(TAG, "Updated face location: x=%d, y=%d", m_last_face_location.x, m_last_face_location.y);
                            }
                        } else {
                            ESP_LOGW(TAG, "Invalid size for FACE_TRACE params. Expected %d, got %d.", 
                                     sizeof(FaceLocation), received_cmd.params.size());
                        }
                        xSemaphoreGive(m_actions_mutex); // Release mutex before breaking
                        break;
                    }

                    default: {
                        // Add new action to the active list
                        if (m_gait_command_map.count(received_cmd.motion_type)) {
                            const std::string& action_name = m_gait_command_map.at(received_cmd.motion_type);
                            const RegisteredAction* action_to_add = m_action_manager.get_action(action_name);
                            if (action_to_add) {
                                m_active_actions.push_back(*action_to_add);
                                ESP_LOGI(TAG, "Action '%s' added to active list.", action_to_add->name);
                            }
                        } else {
                            ESP_LOGW(TAG, "Unknown motion type: 0x%02X", received_cmd.motion_type);
                        }
                        xSemaphoreGive(m_actions_mutex);
                        break;
                    }
                }
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

// --- Face Tracking Task ---
void MotionController::face_tracking_task() {
    ESP_LOGI(TAG, "Face tracking task running...");

    const int control_period_ms = 50; // 20Hz control rate
    const float Kp = 0.1, Ki = 0.01, Kd = 0.05; // PID gains

    // Screen center
    const int screen_center_x = 320 / 2;
    const int screen_center_y = 240 / 2;

    // Initial servo positions (center)
    float pan_angle = 90.0f;
    float tilt_angle = 90.0f;

    while (1) {
        FaceLocation current_face_location;

        // Get the latest face location data safely
        if (xSemaphoreTake(m_face_data_mutex, portMAX_DELAY) == pdTRUE) {
            current_face_location = m_last_face_location;
            xSemaphoreGive(m_face_data_mutex);
        }

        if (current_face_location.detected) {
            // --- PID Calculation for PAN (Left/Right) ---
            int error_pan = screen_center_x - (current_face_location.x + current_face_location.w / 2);
            m_pid_pan_integral += error_pan;
            float derivative_pan = error_pan - m_pid_pan_error_last;
            float output_pan = Kp * error_pan + Ki * m_pid_pan_integral + Kd * derivative_pan;
            m_pid_pan_error_last = error_pan;

            // --- PID Calculation for TILT (Up/Down) ---
            int error_tilt = screen_center_y - (current_face_location.y + current_face_location.h / 2);
            m_pid_tilt_integral += error_tilt;
            float derivative_tilt = error_tilt - m_pid_tilt_error_last;
            float output_tilt = Kp * error_tilt + Ki * m_pid_tilt_integral + Kd * derivative_tilt;
            m_pid_tilt_error_last = error_tilt;

            // Update servo angles
            pan_angle += output_pan;
            tilt_angle -= output_tilt; // Tilt is often inverted

            // Clamp angles to valid servo range (e.g., 0-180)
            if (pan_angle < 0) pan_angle = 0;
            if (pan_angle > 180) pan_angle = 180;
            if (tilt_angle < 0) tilt_angle = 0;
            if (tilt_angle > 180) tilt_angle = 180;

            // Apply angles to servos
            m_servo_driver.set_angle(m_joint_channel_map[static_cast<uint8_t>(ServoChannel::HEAD_PAN)], static_cast<int>(pan_angle));
            m_servo_driver.set_angle(m_joint_channel_map[static_cast<uint8_t>(ServoChannel::HEAD_TILT)], static_cast<int>(tilt_angle));

        } else {
            // Optional: If no face is detected, you could slowly return to center
            // For now, we do nothing and hold the last position.
            // Reset integral term when no face is detected to prevent windup
            m_pid_pan_integral = 0;
            m_pid_tilt_integral = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(control_period_ms));
    }
}

