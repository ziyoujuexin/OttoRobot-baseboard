#include "MotionController.hpp"
#include "esp_log.h"
#include <cmath>
#include <string.h>
#include <algorithm>
#include <queue>

#define PI 3.1415926

static const char* TAG = "MotionController";

static float apply_easing(float t_linear, EasingType type) {
    switch (type) {
        case EasingType::EASE_IN_QUAD:
            return t_linear * t_linear;
        case EasingType::EASE_OUT_QUAD:
            return 1.0f - (1.0f - t_linear) * (1.0f - t_linear);
        case EasingType::EASE_IN_OUT_QUAD:
            return t_linear < 0.5f ? 2.0f * t_linear * t_linear : 1.0f - powf(-2.0f * t_linear + 2.0f, 2.0f) / 2.0f;
        case EasingType::LINEAR:
        default:
            return t_linear;
    }
}

static std::queue<motion_command_t> s_turning_queue;

// --- Constructor / Destructor ---
MotionController::MotionController(Servo& servo_driver, ActionManager& action_manager) 
    : m_servo_driver(servo_driver), 
      m_action_manager(action_manager),
      m_motion_queue(NULL),
      m_actions_mutex(NULL),
      m_interrupt_flag(false),
      m_increment_was_limited_last_cycle(false) 
{
    m_decision_maker = std::make_unique<DecisionMaker>(*this);
}

MotionController::~MotionController() {
    if (m_motion_queue != NULL) {
        vQueueDelete(m_motion_queue);
    }
    if (m_actions_mutex != NULL) {
        vSemaphoreDelete(m_actions_mutex);
    }
    if (m_face_location_queue != NULL) {
        vQueueDelete(m_face_location_queue);
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
    m_gait_command_map[MOTION_FACE_TRACE] = "face_trace";
    m_gait_command_map[MOTION_HAPPY] = "happy";
    m_gait_command_map[MOTION_SAD] = "sad";
    m_gait_command_map[MOTION_SILLY] = "silly";
    m_gait_command_map[MOTION_FUNNY] = "funny";
    m_gait_command_map[MOTION_LAUGHING] = "laughing";
    m_gait_command_map[MOTION_ANGRY] = "angry";
    m_gait_command_map[MOTION_CRYING] = "crying";
    m_gait_command_map[MOTION_SURPRISED] = "surprised";
    m_gait_command_map[MOTION_THINKING] = "thinking";

    // --- 修正特殊动作的映射 ---
    m_gait_command_map[MOTION_TRACKING_L] = "tracking_L";
    m_gait_command_map[MOTION_TRACKING_R] = "tracking_R";
    

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

    m_face_location_queue = xQueueCreate(5, sizeof(FaceLocation)); // Queue size 5, adjust as needed
    if (m_face_location_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create face location queue");
        return;
    }

    // Initialize PID controller variables
    m_pid_pan_error_last = 0;
    m_pid_tilt_error_last = 0;
    m_pan_offset = 0.0f;
    m_tilt_offset = 0.0f;
    m_last_tracking_turn_end_time = 0;
    m_is_head_frozen = false;

    // Initialize Face Tracking Action state
    m_is_tracking_active = false;
    m_head_tracking_action = ActionInstance(); // Value-initialize instead of memset on non-trivial type
    strncpy(m_head_tracking_action.action.name, "head_track", MOTION_NAME_MAX_LEN - 1);
    m_head_tracking_action.action.name[MOTION_NAME_MAX_LEN - 1] = '\0';
    m_head_tracking_action.action.type = ActionType::GAIT_PERIODIC;
    m_head_tracking_action.action.is_atomic = false;
    m_head_tracking_action.action.default_steps = 1; // Will run continuously
    m_head_tracking_action.action.gait_period_ms = 1000;

    motion_params_t base_wave = {};
    for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
        base_wave.offset[i] = 0.0f;
        base_wave.amplitude[i] = 0.01f; // Very small amplitude to allow fine control
        base_wave.phase_diff[i] = 0.0f;
    }
    m_head_tracking_action.action.harmonic_terms.push_back(base_wave);
    m_head_tracking_action.action.easing_type = EasingType::LINEAR;

    m_decision_maker->start(); // Start the new decision maker task

    xTaskCreate(start_task_wrapper, "motion_engine_task", 8192, this, 5, NULL);
    xTaskCreate(start_mixer_task_wrapper, "motion_mixer_task", 4096, this, 6, NULL); // Higher priority for mixer
    xTaskCreate(start_face_tracking_task_wrapper, "face_tracking_task", 4096, this, 5, NULL);
    ESP_LOGI(TAG, "Motion Controller initialized and tasks started.");
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

bool MotionController::queue_face_location(const FaceLocation& face_loc) {
    if (xQueueSend(m_face_location_queue, &face_loc, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Face location queue is full. Data dropped.");
        return false;
    }
    return true;
}

motion_command_t MotionController::get_current_command() {
    motion_command_t current_cmd = {0, {}};
    if (xSemaphoreTake(m_actions_mutex, portMAX_DELAY) == pdTRUE) {
        if (!m_active_actions.empty()) {
            const RegisteredAction& action = m_active_actions.front();
            for (const auto& pair : m_gait_command_map) {
                if (pair.second == action.name) {
                    current_cmd.motion_type = pair.first;
                    break;
                }
            }
        }
        xSemaphoreGive(m_actions_mutex);
    }
    return current_cmd;
}

// --- Core Engine Task (Dispatcher) ---
void MotionController::motion_engine_task() {
    ESP_LOGI(TAG, "Motion engine (dispatcher) task running...");
    motion_command_t received_cmd;
    while (1) {
        if (xQueueReceive(m_motion_queue, &received_cmd, portMAX_DELAY)) {
            
            if (m_interrupt_flag.load() && received_cmd.motion_type == MOTION_STOP) {
                ESP_LOGW(TAG, "STOP command received. Clearing all actions and queue.");
                if (xSemaphoreTake(m_actions_mutex, portMAX_DELAY) == pdTRUE) {
                    m_active_actions.clear();
                    xSemaphoreGive(m_actions_mutex);
                }
                xQueueReset(m_motion_queue);
                xQueueReset(m_face_location_queue); // Clear face location queue
                m_is_tracking_active.store(false); // Deactivate face tracking
                m_interrupt_flag.store(false);
                home();
                continue;
            }

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
                    continue;
                }

                switch (received_cmd.motion_type) {
                    case MOTION_WAKE_DETECT: {break;} // do nothing, just avoid warnings
                    case 0xF0: // MOTION_SERVO_CONTROL
                    {
                        // Check for valid parameter sizes: (channel, angle) or (channel, angle, duration)
                        if (received_cmd.params.size() != (sizeof(uint8_t) + sizeof(float)) &&
                            received_cmd.params.size() != (sizeof(uint8_t) + sizeof(float) + sizeof(uint32_t))) {
                            ESP_LOGE(TAG, "Invalid params size for SERVO_CONTROL: %zu", received_cmd.params.size());
                            break;
                        }

                        // Parse channel and angle
                        uint8_t channel = received_cmd.params[0];
                        float angle;
                        memcpy(&angle, &received_cmd.params[1], sizeof(float));

                        // Parse optional duration, with a default of 1000ms
                        uint32_t duration_ms = 1000;
                        if (received_cmd.params.size() == (sizeof(uint8_t) + sizeof(float) + sizeof(uint32_t))) {
                            memcpy(&duration_ms, &received_cmd.params[sizeof(uint8_t) + sizeof(float)], sizeof(uint32_t));
                        }

                        ESP_LOGI(TAG, "Received SERVO_CONTROL for channel %d to angle %.1f, duration %u ms", channel, angle, (unsigned int)duration_ms);

                        if (channel >= static_cast<uint8_t>(ServoChannel::SERVO_COUNT)) {
                            ESP_LOGE(TAG, "Invalid channel for SERVO_CONTROL: %d", channel);
                            break;
                        }

                        // Create a temporary action to control a single servo
                        RegisteredAction servo_action;
                        // memset(&servo_action, 0, sizeof(RegisteredAction));
                        servo_action = RegisteredAction(); // Value-initialize instead of memset on non-trivial type
                        
                        // Generate a unique name to allow overriding
                        snprintf(servo_action.name, MOTION_NAME_MAX_LEN, "servo_ctrl_%d", channel);
                        
                        servo_action.type = ActionType::GAIT_PERIODIC;
                        servo_action.is_atomic = false; // Allow other actions to run
                        servo_action.default_steps = 1; // Action runs for one "step"
                        servo_action.gait_period_ms = duration_ms; // The "step" duration is the hold time

                        motion_params_t servo_params = {};
                        // Set offset for the target servo. Final Angle = 90 + offset.
                        servo_params.offset[channel] = angle - 90.0f;
                        servo_action.harmonic_terms.push_back(servo_params);
                        servo_action.easing_type = EasingType::LINEAR;

                        // Remove any existing temporary action for the same servo channel to override it
                        m_active_actions.erase(
                            std::remove_if(m_active_actions.begin(), m_active_actions.end(),
                                [&](const RegisteredAction& action) {
                                    return strcmp(action.name, servo_action.name) == 0;
                                }),
                            m_active_actions.end()
                        );

                        // Add the new action to the active list
                        m_active_actions.push_back(servo_action);
                        ESP_LOGI(TAG, "Added temporary servo control action '%s'.", servo_action.name);

                        break;
                    }
                    case MOTION_FACE_TRACE: { 
                        bool is_already_active = false;
                        for(auto const& action : m_active_actions) {
                            if(strcmp(action.name, "head_track") == 0) {
                                is_already_active = true;
                                break;
                            }
                            if(strcmp(action.name, "tracking_L") == 0 || strcmp(action.name, "tracking_R") == 0) {
                                queue_command({MOTION_STOP, {}});
                                break;
                            }
                        }
                        if (!is_already_active) {
                            m_active_actions.push_back(m_head_tracking_action.action);
                            ESP_LOGI(TAG, "Face tracking action activated.");
                        }
                        is_active = true;
                        break;
                    }
                    default: {
                        if (m_gait_command_map.count(received_cmd.motion_type)) {
                            const std::string& action_name = m_gait_command_map.at(received_cmd.motion_type);
                            bool command_handled = false;

                            if (action_name == "tracking_L" || action_name == "tracking_R") {
                                bool is_turning_active = false;
                                for (const auto& action : m_active_actions) {
                                    if (strcmp(action.name, "tracking_L") == 0 || strcmp(action.name, "tracking_R") == 0) {
                                        is_turning_active = true;
                                        break;
                                    }
                                }
                                if (is_turning_active) {
                                    ESP_LOGI(TAG, "A turning action is active. Queuing '%s'.", action_name.c_str());
                                    s_turning_queue.push(received_cmd);
                                    command_handled = true;
                                }
                            }

                            if (!command_handled) {
                                // Check if the action already exists in the active list
                                bool action_already_exists = false;
                                for (const auto& existing_action : m_active_actions) {
                                    if (strcmp(existing_action.name, action_name.c_str()) == 0) {
                                        action_already_exists = true;
                                        ESP_LOGW(TAG, "Action '%s' is already active. Ignoring command.", action_name.c_str());
                                        break;
                                    }
                                }

                                if (!action_already_exists) {
                                    const RegisteredAction* action_to_add = m_action_manager.get_action(action_name);
                                    if (action_to_add && action_to_add->name[0] == '\0') {
                                        ESP_LOGW(TAG, "Action '%s' is empty. Ignoring command.", action_name.c_str());
                                        ESP_LOGW(TAG, "Trying to re-register default actions.");
                                        m_action_manager.register_default_actions(true);
                                        action_to_add = m_action_manager.get_action(action_name);
                                    }
                                    if (action_to_add) {
                                        // If starting a body-moving action, freeze the head to prevent conflict.
                                        if (strcmp(action_name.c_str(), "tracking_L") == 0 ||
                                            strcmp(action_name.c_str(), "tracking_R") == 0 ||
                                            strcmp(action_name.c_str(), "walk_forward") == 0 ||
                                            strcmp(action_name.c_str(), "walk_backward") == 0 ||
                                            strcmp(action_name.c_str(), "turn_left") == 0 ||
                                            strcmp(action_name.c_str(), "turn_right") == 0)
                                        {
                                            m_is_head_frozen.store(true);
                                        }
                                        m_active_actions.push_back(*action_to_add);
                                        is_active = true; // Set active flag
                                        ESP_LOGI(TAG, "Action '%s' added to active list.", action_to_add->name);
                                    }
                                }
                            }
                        } else {
                            ESP_LOGW(TAG, "Unknown motion type: 0x%02X", received_cmd.motion_type);
                        }
                        break;
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
    
    std::map<std::string, int> action_frame_counters;

    while (1) {
        float final_angles[GAIT_JOINT_COUNT] = {0};
        for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
            final_angles[i] = -1.0f; // -1 indicates not set
        }

        bool needs_homing = false;

        if (xSemaphoreTake(m_actions_mutex, portMAX_DELAY) == pdTRUE) {
            
            bool is_turning = false;
            for (const auto& action : m_active_actions) {
                if (strcmp(action.name, "tracking_L") == 0 || strcmp(action.name, "tracking_R") == 0) {
                    is_turning = true;
                    break;
                }
            }

            size_t size_before = m_active_actions.size();
            m_active_actions.erase(
                std::remove_if(m_active_actions.begin(), m_active_actions.end(),
                    [&](RegisteredAction& action) {
                        if (strcmp(action.name, "head_track") == 0) {
                            return false; // Never expires
                        }
                        int frames_per_gait = action.gait_period_ms / control_period_ms;
                        int total_frames = action.default_steps * frames_per_gait;
                        if (action_frame_counters[action.name] >= total_frames) {
                            ESP_LOGI(TAG, "Action '%s' finished and removed.", action.name);

                            // Unfreeze head if a body-moving action has finished.
                            // This assumes only one such action runs at a time.
                            if (strcmp(action.name, "tracking_L") == 0 ||
                                strcmp(action.name, "tracking_R") == 0 ||
                                strcmp(action.name, "walk_forward") == 0 ||
                                strcmp(action.name, "walk_backward") == 0 ||
                                strcmp(action.name, "turn_left") == 0 ||
                                strcmp(action.name, "turn_right") == 0)
                            {
                                m_is_head_frozen.store(false);
                            }

                            // Special cleanup just for body turning actions
                            if (strcmp(action.name, "tracking_L") == 0 || strcmp(action.name, "tracking_R") == 0) {
                                m_last_tracking_turn_end_time = esp_timer_get_time();

                                std::vector<ServoChannel> head_servos = {
                                    ServoChannel::HEAD_PAN,
                                    ServoChannel::HEAD_TILT
                                };
                                home(HomeMode::Blacklist, head_servos);

                                if (!s_turning_queue.empty()) {
                                    motion_command_t next_turn = s_turning_queue.front();
                                    s_turning_queue.pop();
                                    ESP_LOGI(TAG, "Starting next queued turning action.");
                                    queue_command(next_turn);
                                }
                            }

                            action_frame_counters.erase(action.name);
                            return true; // Remove this action
                        }
                        return false;
                    }),
                m_active_actions.end()
            );
            if (size_before > 0 && m_active_actions.empty()) {
                ESP_LOGI(TAG, "All actions completed.");
                needs_homing = false; // Explicitly set to false, was true
                is_active = false; // Set inactive flag
            }

            for (auto& action : m_active_actions) {
                if (action.gait_period_ms == 0) continue;

                int frames_per_gait = action.gait_period_ms / control_period_ms;
                if (frames_per_gait == 0) continue;

                int current_frame = action_frame_counters[action.name];
                float t_linear = (float)(current_frame % frames_per_gait) / frames_per_gait;
                float t_warped = apply_easing(t_linear, action.easing_type);

                bool is_walk_action = (strcmp(action.name, "walk_forward") == 0 || strcmp(action.name, "walk_backward") == 0);

                for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                    if (final_angles[i] >= 0.0f) { continue; }

                    bool is_leg_joint = (i == static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE) ||
                                         i == static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE) ||
                                         i == static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT) ||
                                         i == static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT));

                    if (is_turning && is_walk_action && is_leg_joint) {
                        continue;
                    }

                    if (action.harmonic_terms.empty()) continue;

                    float base_offset = action.harmonic_terms[0].offset[i];
                    float wave_component = 0.0f;
                    int k = 1;

                    for (const auto& term : action.harmonic_terms) {
                        float amp   = term.amplitude[i];
                        float phase = term.phase_diff[i];

                        if (std::abs(amp) > 0.01f) {
                            wave_component += amp * sinf(k * 2 * PI * t_warped + phase);
                        }
                        k++;
                    }

                    float angle = 90.0f + base_offset + wave_component;

                    if (angle < 0) angle = 0;
                    if (angle > 180) angle = 180;
                    final_angles[i] = angle;
                }
                
                action_frame_counters[action.name]++;
            }
            xSemaphoreGive(m_actions_mutex);
        }

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

void MotionController::home(HomeMode mode, const std::vector<ServoChannel>& channels) {
    if(mode != HomeMode::All)
        ESP_LOGI(TAG, "Homing servos with specified mode...");
    for (uint8_t i = 0; i < static_cast<uint8_t>(ServoChannel::SERVO_COUNT); ++i) {
        ServoChannel current_channel = static_cast<ServoChannel>(i);
        bool should_home = false;

        switch (mode) {
            case HomeMode::All:
                should_home = true;
                break;
            case HomeMode::Whitelist:
                if (std::find(channels.begin(), channels.end(), current_channel) != channels.end()) {
                    should_home = true;
                }
                break;
            case HomeMode::Blacklist:
                if (std::find(channels.begin(), channels.end(), current_channel) == channels.end()) {
                    should_home = true;
                }
                break;
        }

        if (should_home) {
            m_servo_driver.set_angle(i, 90);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}

void MotionController::set_single_servo(uint8_t channel, uint8_t angle) {
    m_servo_driver.set_angle(channel, angle);
}

// --- Face Tracking Task ---
void MotionController::face_tracking_task() {
    ESP_LOGI(TAG, "Face tracking task running...");

    const int control_period_ms = 50; // 20Hz control rate
    const float Kp = 0.08f, Kd = 0.04f; // PD gains
    const int deadzone_pixels = 5;      // Deadzone in pixels
    const float delta_limit = 10.0f;      // The single-frame movement limit

    const int screen_center_x = 640 / 2;
    const int screen_center_y = 480 / 2;

    FaceLocation last_processed_location = {0, 0, 0, 0, false};

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(control_period_ms));

        bool new_data_received = false;
        FaceLocation current_face_location;

        // Always try to get the latest face location
        if (xQueueReceive(m_face_location_queue, &current_face_location, 0) == pdTRUE) {
            new_data_received = true;
            if (m_decision_maker) {
                m_decision_maker->set_face_location(current_face_location);
            }
            last_processed_location = current_face_location;
        } else {
            // If no new data, use the last known location for decision making, but don't move
            current_face_location = last_processed_location;
        }

        // If face is lost for a while, reset detected state
        if (!current_face_location.detected && last_processed_location.detected) {
             // Potentially add a timer here to only reset after a few frames of no detection
            last_processed_location.detected = false;
        }
        
        bool head_track_is_active = false;
        if (xSemaphoreTake(m_actions_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            for (const auto& action : m_active_actions) {
                if (strcmp(action.name, "head_track") == 0) {
                    head_track_is_active = true;
                    break;
                }
            }
            xSemaphoreGive(m_actions_mutex);
        }

        if (!head_track_is_active || m_is_head_frozen.load()) {
            if (m_is_tracking_active.load()) {
                m_pid_pan_error_last = 0;
                m_pid_tilt_error_last = 0;
                // m_pan_offset = 0.0f;
                // m_tilt_offset = 0.0f;
                m_is_tracking_active.store(false);
            }
            continue;
        }

        // Only perform movement calculations if there is new data
        if (!new_data_received) {
            continue;
        }

        // Update tracking state based on face size
        if (current_face_location.w > 30 && current_face_location.h > 30) {
            m_is_tracking_active.store(true);
        } else {
            m_is_tracking_active.store(false);
        }

        if (!m_is_tracking_active.load()) {
            m_pid_pan_error_last = 0;
            m_pid_tilt_error_last = 0;
            // m_pan_offset = 0.0f;
            // m_tilt_offset = 0.0f;
            m_increment_was_limited_last_cycle = false;
            continue;
        }

        int error_pan = screen_center_x - (current_face_location.x + current_face_location.w / 2);
        if (std::abs(error_pan) < deadzone_pixels) { error_pan = 0; }
        float derivative_pan = error_pan - m_pid_pan_error_last;
        float output_pan = Kp * error_pan + Kd * derivative_pan;
        m_pid_pan_error_last = error_pan;

        int error_tilt = (current_face_location.y + current_face_location.h / 2) - screen_center_y;
        if (std::abs(error_tilt) < deadzone_pixels) { error_tilt = 0; }
        float derivative_tilt = error_tilt - m_pid_tilt_error_last;
        float output_tilt = Kp * 0.6f * error_tilt + Kd * derivative_tilt;
        m_pid_tilt_error_last = error_tilt;

        if (!std::isfinite(output_pan)) { output_pan = 0.0f; }
        if (!std::isfinite(output_tilt)) { output_tilt = 0.0f; }

        bool is_delta_limited_this_cycle = false;
        if (output_pan > delta_limit)  { output_pan = delta_limit;  is_delta_limited_this_cycle = true; }
        if (output_pan < -delta_limit) { output_pan = -delta_limit; is_delta_limited_this_cycle = true; }
        if (output_tilt > delta_limit * 0.6f) { output_tilt = delta_limit * 0.6f; is_delta_limited_this_cycle = true; }
        if (output_tilt < -delta_limit * 0.6f){ output_tilt = -delta_limit * 0.6f; is_delta_limited_this_cycle = true; }

        if (is_delta_limited_this_cycle) {
            m_increment_was_limited_last_cycle = true;
        } else {
            m_increment_was_limited_last_cycle = false;
        }

        m_pan_offset += output_pan;
        m_tilt_offset += output_tilt;

        if (m_pan_offset < -70.0f) { m_pan_offset = -70.0f; }
        if (m_pan_offset > 70.0f)  { m_pan_offset = 70.0f;  }
        if (m_tilt_offset < -40.0f){ m_tilt_offset = -40.0f; }
        if (m_tilt_offset > 40.0f) { m_tilt_offset = 40.0f;  }

        const int64_t COOLDOWN_PERIOD_US = 3000000; // 3 seconds
        bool is_turning = false;
        if (xSemaphoreTake(m_actions_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            for (const auto& action : m_active_actions) {
                if (strcmp(action.name, "tracking_L") == 0 || strcmp(action.name, "tracking_R") == 0) {
                    is_turning = true;
                    break;
                }
            }
            xSemaphoreGive(m_actions_mutex);
        }

        if (!is_turning) {
            int64_t current_time = esp_timer_get_time();
            if ((current_time - m_last_tracking_turn_end_time) > COOLDOWN_PERIOD_US) {
                if (m_pan_offset <= -70.0f) { // At right limit
                    queue_command({MOTION_TRACKING_R, {}});
                    m_pan_offset += 4 * delta_limit;
                } else if (m_pan_offset >= 70.0f) { // At left limit
                    queue_command({MOTION_TRACKING_L, {}});
                    m_pan_offset -= 4 * delta_limit;
                }
            }
        }

        m_head_tracking_action.action.harmonic_terms[0].offset[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = m_pan_offset;
        m_head_tracking_action.action.harmonic_terms[0].offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = m_tilt_offset;
        ESP_LOGI(TAG, "Face tracking offsets updated: Pan=%.2f, Tilt=%.2f", m_pan_offset, m_tilt_offset);
    }
}

// --- Appended Methods ---
bool MotionController::is_body_moving() const
{
    bool moving = false;
    if (xSemaphoreTake(m_actions_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (const auto& action : m_active_actions) {
            if (strcmp(action.name, "walk_forward") == 0 ||
                strcmp(action.name, "walk_backward") == 0 ||
                strcmp(action.name, "turn_L") == 0 ||
                strcmp(action.name, "turn_R") == 0 ||
                strcmp(action.name, "tracking_L") == 0 ||
                strcmp(action.name, "tracking_R") == 0)
            {
                moving = true;
                break;
            }
        }
        xSemaphoreGive(m_actions_mutex);
    }
    return moving;
}

DecisionMaker* MotionController::get_decision_maker() const
{
    return m_decision_maker.get();
}

bool MotionController::is_face_tracking_active() const
{
    bool is_tracking = false;
    if (xSemaphoreTake(m_actions_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (const auto& action : m_active_actions) {
            if (strcmp(action.name, "head_track") == 0) {
                is_tracking = true;
                break;
            }
        }
        xSemaphoreGive(m_actions_mutex);
    }
    return is_tracking;
}
