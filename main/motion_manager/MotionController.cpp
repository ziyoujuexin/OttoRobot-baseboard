#include "MotionController.hpp"
#include "esp_log.h"
#include <cmath>
#include <string.h>
#include <algorithm>

#define PI 3.1415926

#define LIMIT_DELTA 8.0f

static const char* TAG = "MotionController";

// --- Constructor / Destructor ---
MotionController::MotionController(Servo& servo_driver, ActionManager& action_manager) 
    : m_servo_driver(servo_driver), 
      m_action_manager(action_manager),
      m_motion_queue(NULL),
      m_actions_mutex(NULL),
      m_interrupt_flag(false),
      m_increment_was_limited_last_cycle(false) {}

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

    // Initialize Face Tracking Action state
    m_is_tracking_active = false;
    memset(&m_head_tracking_action, 0, sizeof(ActionInstance));
    strncpy(m_head_tracking_action.action.name, "head_track", MOTION_NAME_MAX_LEN - 1);
    m_head_tracking_action.action.type = ActionType::GAIT_PERIODIC;
    m_head_tracking_action.action.is_atomic = false;
    m_head_tracking_action.action.default_steps = 1; // Will run continuously as we don't increment counter
    m_head_tracking_action.action.gait_period_ms = 1000; // Period doesn't matter much with 0 amplitude
    // Initialize offsets to home position (90 degrees), so 0 offset.
    for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
        m_head_tracking_action.action.params.offset[i] = 0.0f;
        m_head_tracking_action.action.params.amplitude[i] = 0.0f;
        m_head_tracking_action.action.params.phase_diff[i] = 0.0f;
    }

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
                        FaceLocation face_loc;
                        if (received_cmd.params.size() == sizeof(FaceLocation)) {
                            memcpy(&face_loc, received_cmd.params.data(), sizeof(FaceLocation));
                            
                            if (xSemaphoreTake(m_face_data_mutex, portMAX_DELAY) == pdTRUE) {
                                m_last_face_location = face_loc;
                                xSemaphoreGive(m_face_data_mutex);
                            }

                            bool face_detected = face_loc.w > 0 && face_loc.h > 0;
                            m_is_tracking_active.store(face_detected);

                            if (face_detected) {
                                // Face detected, ensure tracking action is active
                                bool action_found = false;
                                for (const auto& action : m_active_actions) {
                                    if (strcmp(action.name, m_head_tracking_action.action.name) == 0) {
                                        action_found = true;
                                        break;
                                    }
                                }
                                if (!action_found) {
                                    m_active_actions.push_back(m_head_tracking_action.action);
                                    ESP_LOGI(TAG, "Head tracking action added to active list.");
                                }
                            } else {
                                // Face lost, remove tracking action
                                m_active_actions.erase(
                                    std::remove_if(m_active_actions.begin(), m_active_actions.end(),
                                        [&](const RegisteredAction& action) {
                                            return strcmp(action.name, m_head_tracking_action.action.name) == 0;
                                        }),
                                    m_active_actions.end()
                                );
                                ESP_LOGI(TAG, "Head tracking action removed from active list.");
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
                            // print original cmd vector
                            ESP_LOGI(TAG, "Original command params: ");
                            for (const auto& param : received_cmd.params) {
                                ESP_LOGI(TAG, "0x%02X", param);
                            }
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
                        // Keep head_track action running, it's managed by motion_engine_task
                        if (strcmp(action.name, "head_track") == 0) {
                            return false;
                        }
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

                // Dynamically update head_track action offsets from the shared instance
                if (strcmp(action.name, "head_track") == 0) {
                    action.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = m_head_tracking_action.action.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_PAN)];
                    action.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = m_head_tracking_action.action.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)];
                }

                int frames_per_gait = action.gait_period_ms / control_period_ms;
                if (frames_per_gait == 0) continue;

                int current_frame = action_frame_counters[action.name];
                float t = (float)(current_frame % frames_per_gait) / frames_per_gait;

                for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                    if (final_angles[i] >= 0.0f) { // If not already set by a higher priority action
                        continue;
                    }

                    float amp = action.params.amplitude[i];
                    float offset = action.params.offset[i];
                    bool is_head_track_action = strcmp(action.name, "head_track") == 0;
                    bool is_head_joint = (i == static_cast<uint8_t>(ServoChannel::HEAD_PAN) || i == static_cast<uint8_t>(ServoChannel::HEAD_TILT));

                    // A joint is affected by an action if it has amplitude, or if it's a head joint in the head_track action
                    if (std::abs(amp) > 0.01f || (is_head_track_action && is_head_joint)) {
                        
                        // The sine wave component is only for non-tracking actions with amplitude
                        float wave_component = (!is_head_track_action && std::abs(amp) > 0.01f) 
                                             ? amp * sin(2 * PI * t + action.params.phase_diff[i]) 
                                             : 0.0f;

                        float angle = 90.0f + offset + wave_component;
                        
                        if (angle < 0) angle = 0;
                        if (angle > 180) angle = 180;
                        final_angles[i] = angle;
                    }
                }
                
                // Do not increment frame counter for head_track, so it runs indefinitely
                if (strcmp(action.name, "head_track") != 0) {
                    action_frame_counters[action.name]++;
                }
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

void MotionController::set_single_servo(uint8_t channel, uint8_t angle) {
    m_servo_driver.set_angle(channel, angle);
}

// --- Face Tracking Task ---
void MotionController::face_tracking_task() {
    ESP_LOGI(TAG, "Face tracking task running...");

    const int control_period_ms = 50; // 20Hz control rate
    const float Kp = 0.005f, Kd = 0.2f; // PD gains
    const int deadzone_pixels = 10;      // Deadzone in pixels
    const float delta_limit = 5.0f;      // The single-frame movement limit

    const int screen_center_x = 640 / 2;
    const int screen_center_y = 480 / 2;
    
    float pan_offset = 0.0f;
    float tilt_offset = 0.0f;

    FaceLocation last_processed_location = {0, 0, 0, 0, false};

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(control_period_ms));

        if (!m_is_tracking_active.load()) {
            m_pid_pan_error_last = 0;
            m_pid_tilt_error_last = 0;
            pan_offset = 0.0f;
            tilt_offset = 0.0f;
            m_increment_was_limited_last_cycle = false;
            last_processed_location = {0, 0, 0, 0, false};
            continue;
        }

        FaceLocation current_face_location;
        if (xSemaphoreTake(m_face_data_mutex, portMAX_DELAY) == pdTRUE) {
            current_face_location = m_last_face_location;
            xSemaphoreGive(m_face_data_mutex);
        }

        bool is_same_location = (current_face_location.x == last_processed_location.x &&
                                 current_face_location.y == last_processed_location.y &&
                                 current_face_location.w == last_processed_location.w &&
                                 current_face_location.h == last_processed_location.h &&
                                 current_face_location.detected == last_processed_location.detected);

        if (is_same_location && !m_increment_was_limited_last_cycle) {
            // Data is not new and no pass is available, skip.
            continue;
        }
        
        // Consume the pass if it was available.
        m_increment_was_limited_last_cycle = false;

        last_processed_location = current_face_location;

        // --- PD Calculation for PAN (Left/Right) ---
        int error_pan = screen_center_x - (current_face_location.x + current_face_location.w / 2);
        if (std::abs(error_pan) < deadzone_pixels) {
            error_pan = 0;
        }
        float derivative_pan = error_pan - m_pid_pan_error_last;
        float output_pan = Kp * error_pan + Kd * derivative_pan;
        m_pid_pan_error_last = error_pan;

        // --- PD Calculation for TILT (Up/Down) ---
        int error_tilt = screen_center_y - (current_face_location.y + current_face_location.h / 2);
        if (std::abs(error_tilt) < deadzone_pixels) {
            error_tilt = 0;
        }
        float derivative_tilt = error_tilt - m_pid_tilt_error_last;
        float output_tilt = Kp * error_tilt + Kd * derivative_tilt;
        m_pid_tilt_error_last = error_tilt;

        // Robustness check for NaN or Inf
        if (!std::isfinite(output_pan)) {
            ESP_LOGW(TAG, "PID calculation resulted in invalid number! Resetting pan output.");
            output_pan = 0.0f;
        }
        if (!std::isfinite(output_tilt)) {
            ESP_LOGW(TAG, "PID calculation resulted in invalid number! Resetting tilt output.");
            output_tilt = 0.0f;
        }

        // Clamp the single-frame delta and set the flag if limited
        bool is_delta_limited_this_cycle = false;
        if (output_pan > delta_limit)  { output_pan = delta_limit;  is_delta_limited_this_cycle = true; }
        if (output_pan < -delta_limit) { output_pan = -delta_limit; is_delta_limited_this_cycle = true; }
        if (output_tilt > delta_limit) { output_tilt = delta_limit; is_delta_limited_this_cycle = true; }
        if (output_tilt < -delta_limit){ output_tilt = -delta_limit; is_delta_limited_this_cycle = true; }

        // Grant a pass for the next cycle ONLY if we hit the delta limit on NEW data.
        if (is_delta_limited_this_cycle && !is_same_location) {
            m_increment_was_limited_last_cycle = true;
        }

        // Update servo offsets with the (potentially clamped) delta
        pan_offset += output_pan;
        tilt_offset -= output_tilt; // Tilt is inverted in original logic

        // Clamp final offsets to prevent exceeding hardware limits (this is a safety measure)
        if (pan_offset < -40.0f) { pan_offset = -40.0f; }
        if (pan_offset > 40.0f)  { pan_offset = 40.0f;  }
        if (tilt_offset < -20.0f){ tilt_offset = -20.0f; }
        if (tilt_offset > 70.0f) { tilt_offset = 70.0f;  }

        // Update the shared head tracking action's parameters.
        m_head_tracking_action.action.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = pan_offset;
        m_head_tracking_action.action.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = tilt_offset;
    }
}

