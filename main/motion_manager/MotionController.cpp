#include "MotionController.hpp"
#include "motion_manager/ServoCalibration.hpp"
#include "esp_log.h"
#include <cmath>
#include <string.h>
#include <algorithm>
#include <queue>

#define PI 3.1415926

static const char* TAG = "MotionController";
static std::queue<motion_command_t> s_turning_queue;

// --- Constructor / Destructor ---
MotionController::MotionController(Servo& servo_driver, ActionManager& action_manager) 
    : m_servo_driver(servo_driver), 
      m_action_manager(action_manager),
      m_motion_queue(NULL),
      m_actions_mutex(NULL),
      m_interrupt_flag(false),
      m_increment_was_limited_last_cycle(false),
      m_is_manual_control_active(false), // Initialize new member
      m_manual_control_timeout_us(0),
      m_default_filter_alpha(0.8f), // Initialize default alpha
      m_current_filter_alpha(0.8f) // Initialize current alpha // Initialize new member
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

    // Initialize angle filters first to prevent race condition
    m_angle_filters.resize(GAIT_JOINT_COUNT);
    for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
        m_angle_filters[i].set_alpha(m_default_filter_alpha);
        m_angle_filters[i].reset(ServoCalibration::get_home_pos(static_cast<ServoChannel>(i)));
    }

    m_gait_command_map[MOTION_FORWARD] = "walk_forward";
    m_gait_command_map[MOTION_BACKWARD] = "walk_backward";
    m_gait_command_map[MOTION_LEFT] = "turn_left";
    m_gait_command_map[MOTION_RIGHT] = "turn_right";
    m_gait_command_map[MOTION_WAVE_HAND] = "wave_hand";
    m_gait_command_map[MOTION_WAVE_HELLO] = "wave_hello"; // New lively wave
    m_gait_command_map[MOTION_MOVE_EAR] = "wiggle_ears";
    m_gait_command_map[MOTION_NOD_HEAD] = "nod_head";
    m_gait_command_map[MOTION_SHAKE_HEAD] = "shake_head";
    m_gait_command_map[MOTION_SINGLE_LEG] = "single_leg";
    m_gait_command_map[MOTION_FACE_TRACE] = "face_trace";
    m_gait_command_map[MOTION_HAPPY] = "happy";
    m_gait_command_map[MOTION_SAD] = "look_around"; // Renamed
    m_gait_command_map[MOTION_SILLY] = "silly";
    m_gait_command_map[MOTION_FUNNY] = "funny";
    m_gait_command_map[MOTION_LAUGHING] = "laughing";
    m_gait_command_map[MOTION_ANGRY] = "angry";
    m_gait_command_map[MOTION_CRYING] = "sudden_shock";
    m_gait_command_map[MOTION_SURPRISED] = "curious_ponder";
    m_gait_command_map[MOTION_THINKING] = "thinking";

    // --- 修正特殊动作的映射 ---
    m_gait_command_map[MOTION_TRACKING_L] = "tracking_L";
    m_gait_command_map[MOTION_TRACKING_R] = "tracking_R";
    m_gait_command_map[MOTION_WALK_FORWARD_KF] = "walk_forward_kf";
    m_gait_command_map[MOTION_STARTLE_AND_SIGH] = "startle_and_sigh";
    m_gait_command_map[MOTION_WALK_BACKWARD_KF] = "walk_backward_kf";
    

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
    memset(&m_head_tracking_action, 0, sizeof(ActionInstance));
    strncpy(m_head_tracking_action.action.name, "head_track", MOTION_NAME_MAX_LEN - 1);
    m_head_tracking_action.action.type = ActionType::GAIT_PERIODIC;
    m_head_tracking_action.action.is_atomic = false;
    m_head_tracking_action.action.default_steps = 1; // Will run continuously
    m_head_tracking_action.action.data.gait.gait_period_ms = 1000;
    for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
        m_head_tracking_action.action.data.gait.params.offset[i] = 0.0f;
        m_head_tracking_action.action.data.gait.params.amplitude[i] = 0.01f; // Very small amplitude to allow fine control
        m_head_tracking_action.action.data.gait.params.phase_diff[i] = 0.0f;
    }

    m_decision_maker->start(); // Start the new decision maker task

    xTaskCreatePinnedToCore(start_task_wrapper, "motion_engine_task", 8192, this, 6, NULL, 1);
    xTaskCreatePinnedToCore(start_mixer_task_wrapper, "motion_mixer_task", 4096, this, 7, NULL, 1); // Higher priority for mixer
    xTaskCreatePinnedToCore(start_face_tracking_task_wrapper, "face_tracking_task", 4096, this, 6, NULL, 1);
    ESP_LOGI(TAG, "Motion Controller initialized and tasks started.");
}

bool MotionController::queue_command(const motion_command_t& cmd) {
    if (cmd.motion_type == MOTION_STOP) {
        ESP_LOGI(TAG, "Interrupt flag set by STOP command.");
        m_interrupt_flag.store(true);
    }

    // Clear manual control flag if a new action is queued
    m_is_manual_control_active.store(false);

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
            const ActionInstance& instance = m_active_actions.front();
            for (const auto& pair : m_gait_command_map) {
                if (pair.second == instance.action.name) {
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
                // Clear manual control flag on STOP
                m_is_manual_control_active.store(false);
                continue;
            }

            if (xSemaphoreTake(m_actions_mutex, portMAX_DELAY) == pdTRUE) {
                bool is_blocked_by_atomic = false;
                for (const auto& instance : m_active_actions) {
                    if (instance.action.is_atomic) {
                        ESP_LOGW(TAG, "Ignoring command (0x%02X) because atomic action '%s' is running.", 
                                 received_cmd.motion_type, instance.action.name);
                        is_blocked_by_atomic = true;
                        break;
                    }
                }
                
                if (is_blocked_by_atomic) {
                    xSemaphoreGive(m_actions_mutex);
                    continue;
                }

                // Clear manual control flag if a new action is about to be added
                m_is_manual_control_active.store(false);

                // Helper to create and add a new action instance
                auto add_new_action = [&](const RegisteredAction* action_template) {
                    if (!action_template) return;

                    // If starting a body-moving action, freeze the head to prevent conflict.
                    if (is_body_moving(*action_template))
                    {
                        m_is_head_frozen.store(true);
                    }

                    ActionInstance new_instance = {};
                    new_instance.action = *action_template;
                    new_instance.remaining_steps = action_template->default_steps;
                    new_instance.start_time_ms = esp_timer_get_time() / 1000;

                    if (new_instance.action.type == ActionType::KEYFRAME_SEQUENCE) {
                        new_instance.current_keyframe_index = 0;
                        new_instance.transition_start_time_ms = new_instance.start_time_ms;
                        // Initialize start positions to calibrated home for the first transition
                        for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                            new_instance.start_positions[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
                        }
                    }

                    if (strcmp(action_template->name, "walk_forward_kf") == 0) {
                        apply_filter_alpha(0.3f); // Set alpha to 0.3 specifically for walk_forward_kf
                    }

                    m_active_actions.push_back(new_instance);
                    is_active = true; // Set active flag
                    ESP_LOGI(TAG, "Action '%s' added to active list.", action_template->name);
                };

                switch (received_cmd.motion_type) {
                    case MOTION_WAKE_DETECT: {break;} // do nothing, just avoid warnings
                    case 0xF0: // MOTION_SERVO_CONTROL
                    {
                        // This case is now handled by the mixer logic, but we can keep it for compatibility
                        // It will create a temporary gait action, which is not ideal but works.
                        ESP_LOGW(TAG, "MOTION_SERVO_CONTROL is deprecated and will be handled by mixer.");
                        break;
                    }
                    case MOTION_PLAY_MOTION: // 0xD1
                    {
                        if (!received_cmd.params.empty()) {
                            std::string action_name(received_cmd.params.begin(), received_cmd.params.end());
                            ESP_LOGI(TAG, "Received MOTION_PLAY_MOTION for action: '%s'", action_name.c_str());

                            // Check if the action already exists in the active list
                            bool action_already_exists = false;
                            for (const auto& existing_instance : m_active_actions) {
                                if (strcmp(existing_instance.action.name, action_name.c_str()) == 0) {
                                    action_already_exists = true;
                                    ESP_LOGW(TAG, "Action '%s' is already active. Ignoring command.", action_name.c_str());
                                    break;
                                }
                            }

                            if (!action_already_exists) {
                                const RegisteredAction* action_to_add = m_action_manager.get_action(action_name);
                                if (action_to_add) {
                                    add_new_action(action_to_add);
                                } else {
                                    ESP_LOGE(TAG, "Action '%s' not found in manager!", action_name.c_str());
                                }
                            }
                        } else {
                            ESP_LOGW(TAG, "Received MOTION_PLAY_MOTION with no action name.");
                        }
                        break;
                    }
                    case MOTION_FACE_TRACE: { 
                        bool is_already_active = false;
                        for(auto const& instance : m_active_actions) {
                            if(strcmp(instance.action.name, "head_track") == 0) {
                                is_already_active = true;
                                break;
                            }
                            if(strcmp(instance.action.name, "tracking_L") == 0 || strcmp(instance.action.name, "tracking_R") == 0) {
                                queue_command({MOTION_STOP, {}});
                                break;
                            }
                        }
                        if (!is_already_active) {
                            ActionInstance head_instance = m_head_tracking_action;
                            head_instance.start_time_ms = esp_timer_get_time() / 1000;
                            m_active_actions.push_back(head_instance);
                            ESP_LOGI(TAG, "Face tracking action activated.");
                        }
                        is_active = true;
                        break;
                    }
                    default: {
                        if (m_gait_command_map.count(received_cmd.motion_type)) {
                            const std::string& action_name = m_gait_command_map.at(received_cmd.motion_type);
                            
                            // Check if the action already exists in the active list
                            bool action_already_exists = false;
                            for (const auto& existing_instance : m_active_actions) {
                                if (strcmp(existing_instance.action.name, action_name.c_str()) == 0) {
                                    action_already_exists = true;
                                    ESP_LOGW(TAG, "Action '%s' is already active. Ignoring command.", action_name.c_str());
                                    break;
                                }
                            }

                            if (!action_already_exists) {
                                const RegisteredAction* action_to_add = m_action_manager.get_action(action_name);
                                if (action_to_add) {
                                    add_new_action(action_to_add);
                                } else {
                                     ESP_LOGE(TAG, "Action '%s' not found in manager!", action_name.c_str());
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
    const int control_period_ms = 20; // 50Hz control rate

    while (1) {
        uint32_t current_time_ms = esp_timer_get_time() / 1000;

        float final_angles[GAIT_JOINT_COUNT];
        for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
            final_angles[i] = -1.0f; // -1 indicates not set
        }

        if (xSemaphoreTake(m_actions_mutex, portMAX_DELAY) == pdTRUE) {
            
            // Check for manual control timeout
            if (m_is_manual_control_active.load()) {
                if (esp_timer_get_time() > m_manual_control_timeout_us) {
                    m_is_manual_control_active.store(false);
                    ESP_LOGI(TAG, "Manual control timed out. Returning to idle behavior.");
                }
            }

            if (m_active_actions.empty() && !m_is_manual_control_active.load()) { // Only go home if idle AND not in manual control
                is_active = false;
                for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                    final_angles[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
                }
            } else if (!m_active_actions.empty()) { // Process active actions if any
                is_active = true;
                // --- Process all active actions ---
                for (auto& instance : m_active_actions) {
                    // --- Handle Head Tracking Action Separately ---
                    if (strcmp(instance.action.name, "head_track") == 0) {
                        instance.action.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = m_head_tracking_action.action.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_PAN)];
                        instance.action.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = m_head_tracking_action.action.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)];
                    }

                    // --- Switch based on Action Type ---
                    switch (instance.action.type) {
                        case ActionType::GAIT_PERIODIC: {
                            uint32_t period_ms = instance.action.data.gait.gait_period_ms;
                            if (period_ms == 0) continue;

                            float t = (float)((current_time_ms - instance.start_time_ms) % period_ms) / period_ms;

                            for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                                if (final_angles[i] >= 0.0f) continue; // Don't override already set angles

                                float amp = instance.action.data.gait.params.amplitude[i];
                                float offset = instance.action.data.gait.params.offset[i];
                                
                                if (std::abs(amp) > 0.01f || std::abs(offset) > 0.01f) {
                                    float wave_component = (std::abs(amp) > 0.01f) 
                                                         ? amp * sin(2 * PI * t + instance.action.data.gait.params.phase_diff[i]) 
                                                         : 0.0f;
                                    
                                    float home_pos = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
                                    float angle = home_pos + offset + wave_component;
                                    const auto& limit = ServoCalibration::limits[i];
                                    final_angles[i] = std::max(limit.min, std::min(limit.max, angle));
                                }
                            }
                            break;
                        }

                        case ActionType::KEYFRAME_SEQUENCE: {
                            const auto& keyframe_data = instance.action.data.keyframe;
                            if (keyframe_data.frame_count == 0) continue;

                            // Get current and next keyframes
                            const auto& target_frame = keyframe_data.frames[instance.current_keyframe_index];
                            uint32_t transition_duration = target_frame.transition_time_ms;
                            if (transition_duration == 0) transition_duration = 1; // Avoid division by zero

                            // Calculate interpolation progress (alpha)
                            uint32_t elapsed_in_transition = current_time_ms - instance.transition_start_time_ms;
                            float linear_alpha = (float)elapsed_in_transition / (float)transition_duration;
                            linear_alpha = std::max(0.0f, std::min(1.0f, linear_alpha)); // Clamp alpha

                            // Apply cosine easing for smooth acceleration and deceleration
                            float eased_alpha = 0.5f * (1.0f - cosf(linear_alpha * PI));

                            // Interpolate for each joint
                            for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                                if (final_angles[i] >= 0.0f) continue;

                                float start_pos = instance.start_positions[i];
                                float target_pos = target_frame.positions[i];
                                float angle = start_pos + (target_pos - start_pos) * eased_alpha;
                                
                                const auto& limit = ServoCalibration::limits[i];
                                final_angles[i] = std::max(limit.min, std::min(limit.max, angle));
                            }
                            break;
                        }
                    }
                }
            } else { // If active_actions is empty AND not in manual control, final_angles already set to home
                // If active_actions is empty AND in manual control, do nothing, servos hold last position
            }

            // --- Action Completion and Removal Logic ---
            m_active_actions.erase(
                std::remove_if(m_active_actions.begin(), m_active_actions.end(),
                    [&](ActionInstance& instance) {
                        bool finished = false;
                        if (strcmp(instance.action.name, "head_track") == 0) return false; // Never remove head tracking

                        if (instance.action.type == ActionType::GAIT_PERIODIC) {
                            uint32_t total_duration_ms = instance.action.default_steps * instance.action.data.gait.gait_period_ms;
                            if ((current_time_ms - instance.start_time_ms) >= total_duration_ms) {
                                finished = true;
                            }
                        } else if (instance.action.type == ActionType::KEYFRAME_SEQUENCE) {
                            const auto& kf_data = instance.action.data.keyframe;
                            const auto& target_frame = kf_data.frames[instance.current_keyframe_index];
                            
                            if ((current_time_ms - instance.transition_start_time_ms) >= target_frame.transition_time_ms) {
                                // Current frame transition finished, move to next
                                memcpy(instance.start_positions, target_frame.positions, sizeof(instance.start_positions));
                                instance.current_keyframe_index++;
                                instance.transition_start_time_ms = current_time_ms;

                                if (instance.current_keyframe_index >= kf_data.frame_count) {
                                    // End of sequence
                                    instance.remaining_steps--;
                                    if (instance.remaining_steps == 0) {
                                        finished = true;
                                    } else {
                                        // Loop sequence
                                        instance.current_keyframe_index = 0;
                                    }
                                }
                            }
                        }

                        if (finished) {
                            ESP_LOGI(TAG, "Action '%s' finished and removed.", instance.action.name);
                            if (strcmp(instance.action.name, "tracking_L") == 0 || strcmp(instance.action.name, "tracking_R") == 0) {
                                m_last_tracking_turn_end_time = esp_timer_get_time();
                            }
                            if (is_body_moving(instance.action)) {
                                m_is_head_frozen.store(false);
                                apply_filter_alpha(m_default_filter_alpha); // Revert alpha to default
                            }
                        }
                        return finished;
                    }),
                m_active_actions.end()
            );

            if (m_active_actions.empty()) {
                is_active = false;
            }

            xSemaphoreGive(m_actions_mutex);
        }

        // --- Apply final angles to servos ---
        for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
            if (final_angles[i] >= 0.0f) {
                uint8_t channel = m_joint_channel_map[i];
                float filtered_angle = m_angle_filters[i].apply(final_angles[i]);
                m_servo_driver.set_angle(channel, static_cast<uint16_t>(filtered_angle));
            }
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
            m_servo_driver.set_angle(i, static_cast<uint16_t>(ServoCalibration::get_home_pos(current_channel)));
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}

void MotionController::set_single_servo(uint8_t channel, uint16_t angle) {
    m_servo_driver.set_angle(channel, angle);
    m_is_manual_control_active.store(true);
    m_manual_control_timeout_us = esp_timer_get_time() + 5 * 1000 * 1000; // 5 seconds timeout
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
            for (const auto& instance : m_active_actions) {
                if (strcmp(instance.action.name, "head_track") == 0) {
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
                m_is_tracking_active.store(false);
            }
            continue;
        }

        if (!new_data_received) {
            continue;
        }

        if (current_face_location.w > 30 && current_face_location.h > 30) {
            m_is_tracking_active.store(true);
        } else {
            m_is_tracking_active.store(false);
        }

        if (!m_is_tracking_active.load()) {
            m_pid_pan_error_last = 0;
            m_pid_tilt_error_last = 0;
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
                if (strcmp(action.action.name, "tracking_L") == 0 || strcmp(action.action.name, "tracking_R") == 0) {
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

        m_head_tracking_action.action.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = m_pan_offset;
        m_head_tracking_action.action.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = m_tilt_offset;
    }
}


bool MotionController::is_body_moving(const RegisteredAction& action) const {
    const char* name = action.name;
    return (strcmp(name, "walk_forward") == 0 ||
            strcmp(name, "walk_backward") == 0 ||
            strcmp(name, "turn_left") == 0 ||
            strcmp(name, "turn_right") == 0 ||
            strcmp(name, "tracking_L") == 0 ||
            strcmp(name, "tracking_R") == 0);
}

bool MotionController::is_body_moving() const {
    bool moving = false;
    if (xSemaphoreTake(m_actions_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (const auto& instance : m_active_actions) {
            if (is_body_moving(instance.action)) { // Call the other version of the function
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
        for (const auto& instance : m_active_actions) {
            if (strcmp(instance.action.name, "head_track") == 0) {
                is_tracking = true;
                break;
            }
        }
        xSemaphoreGive(m_actions_mutex);
    }
    return is_tracking;
}

void MotionController::set_filter_alpha(float alpha) {
    if (alpha < 0.0f || alpha > 1.0f) {
        ESP_LOGE(TAG, "Invalid alpha value %.2f. It should be between 0.0 and 1.0.", alpha);
        return;
    }
    for (auto& filter : m_angle_filters) {
        filter.set_alpha(alpha);
    }
    m_default_filter_alpha = alpha; // Update default alpha
    m_current_filter_alpha = alpha; // Update current alpha
    ESP_LOGI(TAG, "Set EMA filter alpha to %.2f for all joints.", alpha);
}

void MotionController::apply_filter_alpha(float alpha) {
    if (alpha < 0.0f || alpha > 1.0f) {
        ESP_LOGE(TAG, "Invalid alpha value %.2f. It should be between 0.0 and 1.0.", alpha);
        return;
    }
    if (m_current_filter_alpha != alpha) {
        for (auto& filter : m_angle_filters) {
            filter.set_alpha(alpha);
        }
        m_current_filter_alpha = alpha;
        ESP_LOGI(TAG, "Dynamically set EMA filter alpha to %.2f.", alpha);
    }
}

