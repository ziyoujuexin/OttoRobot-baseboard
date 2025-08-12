#include "MotionController.hpp"
#include "esp_log.h"
#include <cmath>

#define PI 3.1415926
static const char* TAG = "MotionController";

// --- Constructor / Destructor ---
MotionController::MotionController(Servo& servo_driver) 
    : m_servo_driver(servo_driver), 
      m_motion_queue(NULL),
      m_interrupt_flag(false) {}

MotionController::~MotionController() {
    if (m_motion_queue != NULL) {
        vQueueDelete(m_motion_queue);
    }
}

void MotionController::init_joint_channel_map() {
    // Initialize all servo channels with an identity mapping (channel_id -> channel_id).
    // This ensures that every possible servo has a valid channel defined before
    // being used in gait calculations, fixing the uninitialized read error.
    for (uint8_t i = 0; i < static_cast<uint8_t>(ServoChannel::SERVO_COUNT); ++i) {
        m_joint_channel_map[i] = i;
    }

    // You can still override specific channels here if you need to remap them.
    // For example: m_joint_channel_map[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 15;

    ESP_LOGI(TAG, "Joint-to-Channel map identity-initialized for %d channels.", static_cast<int>(ServoChannel::SERVO_COUNT));
}

// --- Public Methods ---
void MotionController::init() {
    init_joint_channel_map();

    // Map gait commands to action names
    m_gait_command_map[MOTION_FORWARD] = "walk_forward";
    m_gait_command_map[MOTION_BACKWARD] = "walk_backward";
    m_gait_command_map[MOTION_LEFT] = "turn_left";
    m_gait_command_map[MOTION_RIGHT] = "turn_right";

    m_motion_queue = xQueueCreate(10, sizeof(motion_command_t));
    if (m_motion_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create motion queue");
        return;
    }

    m_storage = std::make_unique<MotionStorage>();
    if (!m_storage->init()) {
        ESP_LOGE(TAG, "Failed to initialize MotionStorage");
        return;
    }

    register_default_actions();
    // register_default_groups(); // Demo groups not implemented yet

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

            // Check if the command is a registered gait action
            if (m_gait_command_map.count(received_cmd.motion_type)) {
                const std::string& action_name = m_gait_command_map[received_cmd.motion_type];
                if (m_action_cache.count(action_name)) {
                    execute_action(m_action_cache[action_name]);
                } else {
                    ESP_LOGE(TAG, "Action '%s' not found in cache.", action_name.c_str());
                }
                continue; // Skip the switch statement
            }

            // Handle non-gait and other special commands
            switch (received_cmd.motion_type) {
                case MOTION_STOP:
                    home();
                    break;
                // wave_hand and move_ear are kept separate for now
                case MOTION_WAVE_HAND:
                    wave_hand();
                    break;
                case MOTION_MOVE_EAR:
                    move_ear();
                    break;
                case MOTION_RUN_DEMO_GROUP: {
                    ESP_LOGI(TAG, "Executing demo group...");
                    if (m_group_cache.count("demo_sequence")) {
                        const auto& group = m_group_cache["demo_sequence"];
                        for (int i = 0; i < group.action_count; ++i) {
                            if (m_interrupt_flag.load()) {
                                ESP_LOGI(TAG, "Group execution interrupted.");
                                break;
                            }
                            if (strcmp(group.action_names[i], "wave_hand") == 0) {
                                wave_hand();
                            } else if (m_action_cache.count(group.action_names[i])) {
                                const auto& action_to_run = m_action_cache[group.action_names[i]];
                                execute_action(action_to_run);
                            } else {
                                ESP_LOGW(TAG, "Action '%s' in group 'demo_sequence' not found in cache.", group.action_names[i]);
                            }
                        }
                    } else {
                        ESP_LOGE(TAG, "Group 'demo_sequence' not found in cache.");
                    }
                    break;
                }
                default:
                    ESP_LOGW(TAG, "Unknown motion type: 0x%02X", received_cmd.motion_type);
                    break;
            }
        }
    }
}

// --- Action Execution ---
void MotionController::execute_action(const RegisteredAction& action) {
    print_action_details(action);
    if (action.type == ActionType::GAIT_PERIODIC) {
        execute_gait(action);
    } 
}

void MotionController::execute_gait(const RegisteredAction& action) {
    const int control_period_ms = 20; // 50Hz control rate
    if (action.gait_period_ms == 0) {
        ESP_LOGE(TAG, "Gait period cannot be zero.");
        return;
    }

    // Correctly calculate total frames and frames per gait cycle
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

        // Correctly calculate t as the number of cycles elapsed
        float t = (float)frame / frames_per_gait;

        for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
            float amp = action.params.amplitude[i];
            // Skip servos with no amplitude to save computation
            if (std::abs(amp) < 0.01f) {
                continue;
            }
            float offset = action.params.offset[i];
            float phase = action.params.phase_diff[i];

            float angle = 90.0f + offset + amp * sin(2 * PI * t + phase);
            
            // Clamp angle to valid range
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

void MotionController::register_default_actions() {
    ESP_LOGI(TAG, "Checking and registering default actions...");

    RegisteredAction temp_action;
    if (m_storage->load_action("walk_forward", temp_action)) {
        ESP_LOGI(TAG, "Default actions found in NVS. Loading from storage.");
        m_action_cache["walk_forward"] = temp_action; // Load into cache
        m_storage->load_action("walk_backward", m_action_cache["walk_backward"]);
        m_storage->load_action("turn_left", m_action_cache["turn_left"]);
        m_storage->load_action("turn_right", m_action_cache["turn_right"]);
        return;
    }

    ESP_LOGI(TAG, "Default actions not found in NVS. Creating them...");

    RegisteredAction forward = {};
    strcpy(forward.name, "walk_forward");
    forward.type = ActionType::GAIT_PERIODIC;
    forward.is_atomic = false;
    forward.default_steps = 4;
    forward.gait_period_ms = 1500; // Set a more reasonable gait period: 1.5 seconds per step

    forward.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = 25;
    forward.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 25;
    forward.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = 20;
    forward.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]  = 20;

    forward.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]    = 0;
    forward.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]   = 0;

    forward.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = 0;
    forward.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 0;
    forward.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = PI / 2;
    forward.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]  = -PI / 2;
    
    m_storage->save_action(forward);
    m_action_cache[forward.name] = forward;

    RegisteredAction backward = forward;
    strcpy(backward.name, "walk_backward");
    backward.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -30;
    backward.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -30;
    m_storage->save_action(backward);
    m_action_cache[backward.name] = backward;

    RegisteredAction left = forward;
    strcpy(left.name, "turn_left");
    left.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = 20;
    left.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -20;
    m_storage->save_action(left);
    m_action_cache[left.name] = left;

    RegisteredAction right = forward;
    strcpy(right.name, "turn_right");
    right.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -20;
    right.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 20;
    m_storage->save_action(right);
    m_action_cache[right.name] = right;

    ESP_LOGI(TAG, "Default actions created and cached.");
}

void MotionController::print_action_details(const RegisteredAction &action) {
    ESP_LOGI(TAG, "[Action Details] Name: %s", action.name);
    ESP_LOGI(TAG, "  - Type: %s", action.type == ActionType::GAIT_PERIODIC ? "Gait Periodic" : "Unknown");
    ESP_LOGI(TAG, "  - Atomic: %s", action.is_atomic ? "Yes" : "No");
    ESP_LOGI(TAG, "  - Steps: %d", (int)action.default_steps);
    ESP_LOGI(TAG, "  - Period: %d ms", (int)action.gait_period_ms);
    
    auto print_float_array = [](const char* prefix, const float* arr) {
        char buffer[256];
        int offset = snprintf(buffer, sizeof(buffer), "  - %s: [", prefix);
        for(int i=0; i<GAIT_JOINT_COUNT; ++i) { // Use GAIT_JOINT_COUNT
            offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%.2f, ", arr[i]);
        }
        if (offset > 2) { 
            buffer[offset-2] = ']';
            buffer[offset-1] = '\0';
        }
        ESP_LOGI(TAG, "%s", buffer);
    };

    print_float_array("Amplitude", action.params.amplitude);
    print_float_array("Offset   ", action.params.offset);
    print_float_array("Phase    ", action.params.phase_diff);
}

void MotionController::home() {
    ESP_LOGI(TAG, "Homing all servos to 90 degrees.");
    m_servo_driver.home_all();
    vTaskDelay(pdMS_TO_TICKS(100)); // 等待舵机归位
}

// TODO: 在未来，这些控制都可以使用GAIT来实现，以此实现平滑的移动
void MotionController::wave_hand() {
    ESP_LOGI(TAG, "Executing wave hand");
    for (int wave = 0; wave < 3; wave++) {
        m_servo_driver.set_angle(static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING), 135);
        m_servo_driver.set_angle(static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING), 45);
        vTaskDelay(pdMS_TO_TICKS(500));
        m_servo_driver.set_angle(static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING), 90);
        m_servo_driver.set_angle(static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING), 90);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    m_servo_driver.set_angle(static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING), 90);
    m_servo_driver.set_angle(static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING), 90);
}

void MotionController::move_ear() {
    ESP_LOGI(TAG, "Executing move ear");
    for (int move = 0; move < 5; move++) {
        m_servo_driver.set_angle(static_cast<uint8_t>(ServoChannel::LEFT_EAR_LIFT), 60);
        m_servo_driver.set_angle(static_cast<uint8_t>(ServoChannel::RIGHT_EAR_LIFT), 120);
        vTaskDelay(pdMS_TO_TICKS(300));
        m_servo_driver.set_angle(static_cast<uint8_t>(ServoChannel::LEFT_EAR_LIFT), 120);
        m_servo_driver.set_angle(static_cast<uint8_t>(ServoChannel::RIGHT_EAR_LIFT), 60);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    m_servo_driver.set_angle(static_cast<uint8_t>(ServoChannel::LEFT_EAR_LIFT), 90);
    m_servo_driver.set_angle(static_cast<uint8_t>(ServoChannel::RIGHT_EAR_LIFT), 90);
}

void MotionController::servo_test(uint8_t channel, uint8_t angle) {
    m_servo_driver.set_angle(channel, angle);
}

// --- NVS Storage Interface ---
bool MotionController::delete_action_from_nvs(const std::string& action_name) {
    ESP_LOGI(TAG, "Attempting to delete action '%s' from NVS...", action_name.c_str());
    bool success = m_storage->delete_action(action_name.c_str());
    if (success) {
        // Also remove from cache if it exists
        if (m_action_cache.count(action_name)) {
            m_action_cache.erase(action_name);
            ESP_LOGI(TAG, "Action '%s' removed from cache.", action_name.c_str());
        }
    }
    return success;
}

bool MotionController::delete_group_from_nvs(const std::string& group_name) {
    ESP_LOGI(TAG, "Attempting to delete group '%s' from NVS...", group_name.c_str());
    bool success = m_storage->delete_group(group_name.c_str());
    if (success) {
        // Also remove from cache if it exists
        if (m_group_cache.count(group_name)) {
            m_group_cache.erase(group_name);
            ESP_LOGI(TAG, "Group '%s' removed from cache.", group_name.c_str());
        }
    }
    return success;
}

std::vector<std::string> MotionController::list_actions_from_nvs() {
    std::vector<std::string> actions;
    m_storage->list_actions(actions);
    ESP_LOGI(TAG, "Found %d actions in NVS.", actions.size());
    return actions;
}

std::vector<std::string> MotionController::list_groups_from_nvs() {
    std::vector<std::string> groups;
    m_storage->list_groups(groups);
    ESP_LOGI(TAG, "Found %d groups in NVS.", groups.size());
    return groups;
}

// --- Real-time Gait Tuning ---
bool MotionController::update_action_properties(const std::string& action_name, bool is_atomic, uint32_t default_steps, uint32_t gait_period_ms) {
    auto it = m_action_cache.find(action_name);
    if (it == m_action_cache.end()) {
        ESP_LOGE(TAG, "Action '%s' not found in cache for property update.", action_name.c_str());
        return false;
    }

    RegisteredAction& action = it->second;
    action.is_atomic = is_atomic;
    action.default_steps = default_steps;
    action.gait_period_ms = gait_period_ms;

    ESP_LOGI(TAG, "Updated properties for action '%s': is_atomic=%s, steps=%d, period=%dms", 
             action_name.c_str(), is_atomic ? "true" : "false", (int)default_steps, (int)gait_period_ms);
    
    // After updating, we can optionally print the details to confirm
    print_action_details(action);

    return true;
}

bool MotionController::tune_gait_parameter(const std::string& action_name, int servo_index, const std::string& param_type, float value) {
    auto it = m_action_cache.find(action_name);
    if (it == m_action_cache.end()) {
        ESP_LOGE(TAG, "Action '%s' not found in cache for tuning.", action_name.c_str());
        return false;
    }

    if (servo_index < 0 || servo_index >= GAIT_JOINT_COUNT) {
        ESP_LOGE(TAG, "Invalid servo index: %d", servo_index);
        return false;
    }

    // Safely access the action using the iterator
    RegisteredAction& action = it->second;

    if (param_type == "amplitude") {
        action.params.amplitude[servo_index] = value;
    } else if (param_type == "offset") {
        action.params.offset[servo_index] = value;
    } else if (param_type == "phase_diff") {
        action.params.phase_diff[servo_index] = value;
    } else {
        ESP_LOGE(TAG, "Unknown parameter type: %s", param_type.c_str());
        return false;
    }

    ESP_LOGI(TAG, "Tuned %s for %s, servo %d: set to %.2f", param_type.c_str(), action_name.c_str(), servo_index, value);
    return true;
}

bool MotionController::save_action_to_nvs(const std::string& action_name) {
    auto it = m_action_cache.find(action_name);
    if (it == m_action_cache.end()) {
        ESP_LOGE(TAG, "Action '%s' not found in cache, cannot save.", action_name.c_str());
        return false;
    }

    ESP_LOGI(TAG, "Saving action '%s' to NVS...", action_name.c_str());
    // Safely access the action using the iterator
    return m_storage->save_action(it->second);
}

std::string MotionController::get_action_params_json(const std::string& action_name) {
    auto it = m_action_cache.find(action_name);
    if (it == m_action_cache.end()) {
        return "{}"; // Return empty JSON object if action not found
    }

    const RegisteredAction& action = it->second;

    // Use std::string for dynamic allocation on the heap to avoid stack overflow
    std::string json_str;
    json_str.reserve(1024); // Increased buffer size for new fields

    char temp_buf[128]; 

    // Add top-level properties
    snprintf(temp_buf, sizeof(temp_buf), 
        "{\"name\":\"%s\",\"is_atomic\":%s,\"default_steps\":%d,\"gait_period_ms\":%d,\"params\":{", 
        action.name, 
        action.is_atomic ? "true" : "false",
        (int)action.default_steps, 
        (int)action.gait_period_ms);
    json_str += temp_buf;

    auto& params = action.params;

    // Amplitude Array
    json_str += "\"amplitude\":[";
    for(int i=0; i<GAIT_JOINT_COUNT; ++i) {
        snprintf(temp_buf, sizeof(temp_buf), "%.2f", params.amplitude[i]);
        json_str += temp_buf;
        if (i < GAIT_JOINT_COUNT - 1) json_str += ",";
    }
    json_str += "],";

    // Offset Array
    json_str += "\"offset\":[";
    for(int i=0; i<GAIT_JOINT_COUNT; ++i) {
        snprintf(temp_buf, sizeof(temp_buf), "%.2f", params.offset[i]);
        json_str += temp_buf;
        if (i < GAIT_JOINT_COUNT - 1) json_str += ",";
    }
    json_str += "],";

    // Phase Difference Array
    json_str += "\"phase_diff\":[";
    for(int i=0; i<GAIT_JOINT_COUNT; ++i) {
        snprintf(temp_buf, sizeof(temp_buf), "%.2f", params.phase_diff[i]);
        json_str += temp_buf;
        if (i < GAIT_JOINT_COUNT - 1) json_str += ",";
    }
    json_str += "]}}"; // End of params object and main object

    return json_str;
}

