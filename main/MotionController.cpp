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
    m_joint_channel_map[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = 10;
    m_joint_channel_map[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 12;
    m_joint_channel_map[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = 11;
    m_joint_channel_map[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]  = 13;
    m_joint_channel_map[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)]    = 6;
    m_joint_channel_map[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)]   = 8;
    m_joint_channel_map[static_cast<uint8_t>(ServoChannel::HEAD_PAN)]          = 5;
    m_joint_channel_map[static_cast<uint8_t>(ServoChannel::HEAD_TILT)]         = 4;
    ESP_LOGI(TAG, "Joint-to-Channel map initialized.");
}

// --- Public Methods ---
void MotionController::init() {
    init_joint_channel_map();

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
    // register_default_groups();

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
            switch (received_cmd.motion_type) {
                case MOTION_STOP:
                    home();
                    break;
                case MOTION_FORWARD:
                    execute_action(m_action_cache["walk_forward"]);
                    break;
                case MOTION_BACKWARD:
                    execute_action(m_action_cache["walk_backward"]);
                    break;
                case MOTION_LEFT:
                    execute_action(m_action_cache["turn_left"]);
                    break;
                case MOTION_RIGHT:
                    execute_action(m_action_cache["turn_right"]);
                    break;
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
    int total_frames = action.default_steps * 20;
    int frame_delay_ms = action.default_speed_ms / 20;

    for (int frame = 0; frame < total_frames; frame++) {
        if (m_interrupt_flag.load()) {
            ESP_LOGI(TAG, "Gait interrupted.");
            break;
        }

        float t = (float)frame / 20.0f;
        for (int i = 0; i < GAIT_JOINT_COUNT; ++i) {
            float amp = action.params.amplitude[i];
            float offset = action.params.offset[i];
            float phase = action.params.phase_diff[i];

            float angle = 90.0f + offset + amp * sin(2 * PI * t + phase);
            if (angle < 0) angle = 0;
            if (angle > 180) angle = 180;

            uint8_t channel = m_joint_channel_map[i];
            m_servo_driver.set_angle(channel, static_cast<int>(angle));
        }
        vTaskDelay(pdMS_TO_TICKS(frame_delay_ms));
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
        m_storage->load_action("walk_forward", m_action_cache["walk_forward"]);
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
    forward.default_speed_ms = 800;

    forward.params.amplitude[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)]  = 30;
    forward.params.amplitude[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] = 30;
    forward.params.amplitude[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)]   = 15;
    forward.params.amplitude[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)]  = 15;

    forward.params.offset[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)]    = 5;
    forward.params.offset[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)]   = -5;

    forward.params.phase_diff[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)]  = 0;
    forward.params.phase_diff[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] = PI;
    forward.params.phase_diff[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)]   = -PI / 2;
    forward.params.phase_diff[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)]  = -PI / 2;
    
    m_storage->save_action(forward);
    m_action_cache[forward.name] = forward;

    RegisteredAction backward = forward;
    strcpy(backward.name, "walk_backward");
    backward.params.amplitude[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)]  = -30;
    backward.params.amplitude[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] = -30;
    m_storage->save_action(backward);
    m_action_cache[backward.name] = backward;

    RegisteredAction left = forward;
    strcpy(left.name, "turn_left");
    left.params.amplitude[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)]  = 20;
    left.params.amplitude[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] = -20;
    m_storage->save_action(left);
    m_action_cache[left.name] = left;

    RegisteredAction right = forward;
    strcpy(right.name, "turn_right");
    right.params.amplitude[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)]  = -20;
    right.params.amplitude[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] = 20;
    m_storage->save_action(right);
    m_action_cache[right.name] = right;

    ESP_LOGI(TAG, "Default actions created and cached.");
}

void MotionController::print_action_details(const RegisteredAction &action) {
    ESP_LOGI(TAG, "[Action Details] Name: %s", action.name);
    ESP_LOGI(TAG, "  - Type: %s", action.type == ActionType::GAIT_PERIODIC ? "Gait Periodic" : "Unknown");
    ESP_LOGI(TAG, "  - Atomic: %s", action.is_atomic ? "Yes" : "No");
    ESP_LOGI(TAG, "  - Steps: %d", (int)action.default_steps);
    ESP_LOGI(TAG, "  - Speed: %d ms", (int)action.default_speed_ms);
    
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

void MotionController::wave_hand() {
    ESP_LOGI(TAG, "Executing wave hand");
    for (int wave = 0; wave < 3; wave++) {
        m_servo_driver.set_angle(static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING), 45);
        m_servo_driver.set_angle(static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING), 135);
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