#include "motion_manager/ActionManager.hpp"
#include "motion_manager/ServoCalibration.hpp"
#include "esp_log.h"
#include <cmath>
#include <cstring>
#include <array>

#define PI 3.1415926
static const char* TAG = "ActionManager";

ActionManager::ActionManager() {}

ActionManager::~ActionManager() {}

void ActionManager::init() {
    m_storage = std::make_unique<MotionStorage>();
    if (!m_storage->init()) {
        ESP_LOGE(TAG, "Failed to initialize MotionStorage");
        return;
    }
    register_default_actions(true); // Force re-creation to bypass NVS
    ESP_LOGI(TAG, "ActionManager initialized.");
}

const RegisteredAction* ActionManager::get_action(const std::string& name) const {
    auto it = m_action_cache.find(name);
    if (it != m_action_cache.end()) {
        return &it->second;
    }
    ESP_LOGE(TAG, "Action '%s' not found in cache.", name.c_str());
    return nullptr;
}

const RegisteredGroup* ActionManager::get_group(const std::string& name) const {
    auto it = m_group_cache.find(name);
    if (it != m_group_cache.end()) {
        return &it->second;
    }
    return nullptr; // Return nullptr if group not found, logging will be handled by the controller
}

void ActionManager::register_default_actions(bool force) {
    ESP_LOGI(TAG, "Checking and registering default actions...");

    if (!force) {
        RegisteredAction temp_walk_forward, temp_wave_hello;
        if (m_storage->load_action("walk_forward", temp_walk_forward) && m_storage->load_action("wave_hello", temp_wave_hello)) {
            ESP_LOGI(TAG, "All default actions (including new ones) found in NVS. Loading from storage.");
            m_action_cache["walk_forward"] = temp_walk_forward;
            m_storage->load_action("walk_backward", m_action_cache["walk_backward"]);
            m_storage->load_action("walk_backward_kf", m_action_cache["walk_backward_kf"]);
            m_storage->load_action("turn_left", m_action_cache["turn_left"]);
            m_storage->load_action("turn_right", m_action_cache["turn_right"]);
            m_storage->load_action("wiggle_ears", m_action_cache["wiggle_ears"]);
            m_storage->load_action("wave_hand", m_action_cache["wave_hand"]);
            m_storage->load_action("nod_head", m_action_cache["nod_head"]);
            m_storage->load_action("shake_head", m_action_cache["shake_head"]);
            m_storage->load_action("single_leg", m_action_cache["single_leg"]);
            m_storage->load_action("happy", m_action_cache["happy"]);
            m_storage->load_action("silly", m_action_cache["silly"]);
            m_storage->load_action("funny", m_action_cache["funny"]);
            m_storage->load_action("very_happy", m_action_cache["very_happy"]);
            m_storage->load_action("angry", m_action_cache["angry"]);
            m_storage->load_action("crying", m_action_cache["crying"]);
            m_storage->load_action("surprised", m_action_cache["surprised"]);
            m_storage->load_action("tracking_L", m_action_cache["tracking_L"]);
            m_storage->load_action("tracking_R", m_action_cache["tracking_R"]);
            m_action_cache["wave_hello"] = temp_wave_hello;
            return;
        }
    }

    ESP_LOGI(TAG, "Default actions not found or outdated. Creating/re-creating them...");

   { // Scope for walk_forward_kf (Generated from periodic gait - Emo-like shuffle)
        RegisteredAction walk_forward_kf = {};
        strcpy(walk_forward_kf.name, "walk_forward_kf");
        walk_forward_kf.type = ActionType::KEYFRAME_SEQUENCE;
        walk_forward_kf.is_atomic = false;
        walk_forward_kf.default_steps = 4; // Loop the full cycle 4 times
        
        auto& kf_data = walk_forward_kf.data.keyframe;
        kf_data.frame_count = 0;

        auto create_home_pos = []() {
            std::array<float, GAIT_JOINT_COUNT> pos;
            for(int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                pos[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
            }
            return pos;
        };

        const int frame_time = 1200 / 16; // 93.75ms

        // Frame 0
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] += 15.00f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] += 15.00f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        // Frame 1
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] += 12.63f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] += 12.63f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] += 13.86f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] += 13.86f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] -= 19.13f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] += 19.13f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        // Frame 2
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] += 23.33f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] += 23.33f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] += 10.61f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] += 10.61f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] -= 35.36f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] += 35.36f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        // Frame 3
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] += 30.48f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] += 30.48f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] += 5.74f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] += 5.74f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] -= 46.19f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] += 46.19f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        // Frame 4
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] += 33.00f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] += 33.00f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] += 0.00f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] += 0.00f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] -= 50.00f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] += 50.00f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        // Frame 5
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] += 30.48f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] += 30.48f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] -= 5.74f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] -= 5.74f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] -= 46.19f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] += 46.19f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        // Frame 6
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] += 23.33f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] += 23.33f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] -= 10.61f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] -= 10.61f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] -= 35.36f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] += 35.36f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        // Frame 7
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] += 12.63f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] += 12.63f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] -= 13.86f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] -= 13.86f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] -= 19.13f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] += 19.13f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        // Frame 8
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] += 0.00f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] += 0.00f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] -= 15.00f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] -= 15.00f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] -= 0.00f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] += 0.00f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        // Frame 9
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] -= 12.63f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] -= 12.63f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] -= 13.86f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] -= 13.86f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] += 19.13f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] -= 19.13f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        // Frame 10
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] -= 23.33f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] -= 23.33f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] -= 10.61f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] -= 10.61f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] += 35.36f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] -= 35.36f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        // Frame 11
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] -= 30.48f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] -= 30.48f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] -= 5.74f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] -= 5.74f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] += 46.19f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] -= 46.19f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        // Frame 12
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] -= 33.00f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] -= 33.00f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] -= 0.00f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] -= 0.00f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] += 50.00f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] -= 50.00f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        // Frame 13
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] -= 30.48f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] -= 30.48f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] += 5.74f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] += 5.74f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] += 46.19f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] -= 46.19f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        // Frame 14
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] -= 23.33f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] -= 23.33f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] += 10.61f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] += 10.61f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] += 35.36f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] -= 35.36f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        // Frame 15
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] -= 12.63f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] -= 12.63f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] += 13.86f;  
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] += 13.86f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] += 19.13f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] -= 19.13f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        m_storage->save_action(walk_forward_kf);
        m_action_cache[walk_forward_kf.name] = walk_forward_kf;
    }


    {
        RegisteredAction forward = {};
        strcpy(forward.name, "walk_forward");
        forward.type = ActionType::GAIT_PERIODIC;
        forward.is_atomic = false;
        forward.default_steps = 4;
        forward.data.gait.gait_period_ms = 1500;
        forward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = 33;
        forward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 33;
        forward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = 15;
        forward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]  = 15;
        forward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)]  = -50;
        forward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)]  = 50;
        forward.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = PI / 2;
        forward.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]  = PI / 2;
        m_storage->save_action(forward);
        m_action_cache[forward.name] = forward;
    }

    {
        RegisteredAction backward = m_action_cache["walk_forward"];
        strcpy(backward.name, "walk_backward");
        backward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -33;
        backward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -33;
        backward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)]  = -60;
        backward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)]  = -60;
        m_storage->save_action(backward);
        m_action_cache[backward.name] = backward;
    }

    { // Scope for turn_left_kf (Shuffle-Turn Logic V2)
        RegisteredAction turn_left_kf = {};
        strcpy(turn_left_kf.name, "turn_left");
        turn_left_kf.type = ActionType::KEYFRAME_SEQUENCE;
        turn_left_kf.is_atomic = false;
        turn_left_kf.default_steps = 4;
        
        auto& kf_data = turn_left_kf.data.keyframe;
        kf_data.frame_count = 0;

        auto create_home_pos = []() {
            std::array<float, GAIT_JOINT_COUNT> pos;
            for(int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                pos[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
            }
            return pos;
        };

        const int frame_time = 1200 / 16; // Faster cycle

        // Parameters for a left shuffle-turn (V2 - Reduced amplitude)
        const float r_leg_rot_amp = 20.0f;  // Right leg swings more
        const float l_leg_rot_amp = -20.0f; // Left leg swings backward less
        const float lift_amp = 20.0f;       // Lift height for both feet
        const float arm_amp = -30.0f;       // Arms swing opposite to body rotation

        for (int i = 0; i < 16; ++i) {
            if (kf_data.frame_count >= MAX_KEYFRAMES_PER_ACTION) break;
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            float theta = (float)i * 2.0f * PI / 16.0f + PI;
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] += r_leg_rot_amp * sin(theta);
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)]  += l_leg_rot_amp * sin(theta);
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] += lift_amp * cos(theta) ; // cos(theta) is sin(theta + PI/2)
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)]  += lift_amp * 1.2 * cos(theta) + 8.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)]  -= arm_amp * sin(theta);
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)]   += arm_amp * sin(theta);

            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        m_storage->save_action(turn_left_kf);
        m_action_cache[turn_left_kf.name] = turn_left_kf;
    }

    { // Scope for turn_right_kf (Shuffle-Turn Logic V2)
        RegisteredAction turn_right_kf = {};
        strcpy(turn_right_kf.name, "turn_right");
        turn_right_kf.type = ActionType::KEYFRAME_SEQUENCE;
        turn_right_kf.is_atomic = false;
        turn_right_kf.default_steps = 4;
        
        auto& kf_data = turn_right_kf.data.keyframe;
        kf_data.frame_count = 0;

        auto create_home_pos = []() {
            std::array<float, GAIT_JOINT_COUNT> pos;
            for(int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                pos[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
            }
            return pos;
        };

        const int frame_time = 1200 / 16; // Faster cycle

        // Parameters for a right shuffle-turn (V2 - Reduced amplitude, mirrored)
        const float r_leg_rot_amp = -10.0f; // Right leg swings backward less
        const float l_leg_rot_amp = 28.0f;  // Left leg swings more
        const float lift_amp = 20.0f;       // Lift height for both feet
        const float arm_amp = 30.0f;        // Arms swing opposite to body rotation

        for (int i = 0; i < 16; ++i) {
            if (kf_data.frame_count >= MAX_KEYFRAMES_PER_ACTION) break;
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            float theta = (float)i * 2.0f * PI / 16.0f;
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] += r_leg_rot_amp * sin(theta);
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)]  += l_leg_rot_amp * sin(theta);
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] += lift_amp * cos(theta) + 5.0f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)]  += lift_amp * cos(theta) + 8.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)]  += arm_amp * sin(theta);
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)]   -= arm_amp * sin(theta);

            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        
        m_storage->save_action(turn_right_kf);
        m_action_cache[turn_right_kf.name] = turn_right_kf;
    }

    {
        RegisteredAction wiggle_ears = {};
        strcpy(wiggle_ears.name, "wiggle_ears");
        wiggle_ears.type = ActionType::GAIT_PERIODIC;
        wiggle_ears.is_atomic = false;
        wiggle_ears.default_steps = 2;
        wiggle_ears.data.gait.gait_period_ms = 1500;
        wiggle_ears.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_LIFT)] = 15;
        wiggle_ears.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_LIFT)] = 15;
        wiggle_ears.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 10;
        wiggle_ears.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 10;
        m_storage->save_action(wiggle_ears);
        m_action_cache[wiggle_ears.name] = wiggle_ears;
    }

    { // Scope for wave_hand
        RegisteredAction wave_hand = {};
        strcpy(wave_hand.name, "wave_hand");
        wave_hand.type = ActionType::KEYFRAME_SEQUENCE;
        wave_hand.is_atomic = false;
        wave_hand.default_steps = 1;
        
        auto& kf_data = wave_hand.data.keyframe;
        kf_data.frame_count = 0;

        auto create_home_pos = []() {
            std::array<float, GAIT_JOINT_COUNT> pos;
            for(int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                pos[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
            }
            return pos;
        };

        // Frame 0: Return to home
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 100;
            auto pos = create_home_pos(); // Return all to home
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        m_storage->save_action(wave_hand);
        m_action_cache[wave_hand.name] = wave_hand;
    }

    {
        RegisteredAction nod_head = {};
        strcpy(nod_head.name, "nod_head");
        nod_head.type = ActionType::GAIT_PERIODIC;
        nod_head.is_atomic = false;
        nod_head.default_steps = 2;
        nod_head.data.gait.gait_period_ms = 1500;
        nod_head.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 10;
        m_storage->save_action(nod_head);
        m_action_cache[nod_head.name] = nod_head;
    }

    {
        RegisteredAction shake_head = m_action_cache["nod_head"];
        strcpy(shake_head.name, "shake_head");
        shake_head.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = 20;
        shake_head.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 0;
        m_storage->save_action(shake_head);
        m_action_cache[shake_head.name] = shake_head;
    }

    {
        RegisteredAction walk_backward_kf = {};
        strcpy(walk_backward_kf.name, "walk_backward_kf");
        walk_backward_kf.type = ActionType::KEYFRAME_SEQUENCE;
        walk_backward_kf.is_atomic = false;
        walk_backward_kf.default_steps = 4; // Loop the full cycle 4 times
        
        auto& kf_data = walk_backward_kf.data.keyframe;
        kf_data.frame_count = 0;

        auto create_home_pos = []() {
            std::array<float, GAIT_JOINT_COUNT> pos;
            for(int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                pos[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
            }
            return pos;
        };

        const float original_rot_amp = 55.0f;
        const float forward_rot_amp = original_rot_amp * 0.90f;
        const float backward_rot_amp = original_rot_amp * 0.80f;
        const float lift_amp = 40.0f;
        const int frame_time = 180;

        // Backward sequence starts here
        // Frame 0: Start by lifting right foot high
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] -= lift_amp; // Right foot at peak lift
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 80.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 1: Right leg 50% backward, Left leg 50% forward, Right foot landing
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] += forward_rot_amp * 0.5f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] += forward_rot_amp * 0.5f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] -= lift_amp * 0.5f; // Right foot moving down to land
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 80.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 2: Right leg full backward, Left leg full forward
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] += forward_rot_amp;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] += forward_rot_amp;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] -= lift_amp * 0.5f; // Left foot pushing off
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 80.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 3: Right leg 50% backward, Left leg 50% forward, Left foot lifting
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] += forward_rot_amp * 0.5f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] += forward_rot_amp * 0.5f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] += lift_amp * 0.5f; // Left foot lifting high
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 80.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 4: Legs neutral, Left foot at max height
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] += lift_amp; // Left foot at peak lift
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 80.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 5: Left leg 50% backward, Right leg 50% forward, Left foot landing
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] -= backward_rot_amp * 0.5f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] -= backward_rot_amp * 0.5f;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] += lift_amp * 0.5f; // Left foot moving down to land
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 80.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 6: Left leg full backward, Right leg full forward
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] -= backward_rot_amp;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] -= backward_rot_amp;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] += lift_amp * 0.5f; // Right foot pushing off
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 80.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 7: Left leg 50% backward, Right leg 50% forward, Right foot lifting
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] -= backward_rot_amp * 0.5f;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] -= backward_rot_amp * 0.5f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] -= lift_amp * 0.5f; // Right foot lifting high
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 80.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        m_storage->save_action(walk_backward_kf);
        m_action_cache[walk_backward_kf.name] = walk_backward_kf;
    }

    {
        RegisteredAction silly = {};
        strcpy(silly.name, "dance");
        silly.type = ActionType::KEYFRAME_SEQUENCE;
        silly.is_atomic = false; // can be interrupted
        silly.default_steps = 2; // Play sequence only once
        
        auto& kf_data = silly.data.keyframe;
        kf_data.frame_count = 0;

        auto create_home_pos = []() {
            std::array<float, GAIT_JOINT_COUNT> pos;
            for(int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                pos[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
            }
            return pos;
        };

        // Keyframe 1: Look Left, Arms Out (Attention!) - 1.0s
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1000;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 95;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 90;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 100;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 85;
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 70;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 30;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_LIFT)] = 30;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Keyframe 2: Look Right, Crouch - 1.3s
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1000;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 110;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 60;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 80;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 120;
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 75;
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 110;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] = 100;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] = 60;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 110;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_LIFT)] = 100;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Keyframe 3: Dance Twist Left (STABILITY V3) - 1.8s
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1500;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 65;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 150;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 150;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 40;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_LIFT)] = 30;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] = 45;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] = 90;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] = 45;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] = 105;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Keyframe 4: Dance Twist Right (V4 COORDINATION FIX) - 1.8s
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1500;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 75;       // Reduced head nod
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 30;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 30;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 110;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_LIFT)] = 100;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] = 110;  // Reduced left leg outward rotation
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] = 110;  // Reduced left leg lift
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] = 135;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] = 90;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Keyframe 5: Shimmy 1 - 1.3s
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1100;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 75;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 110;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 70;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 80;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 110;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 120;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 120;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 50;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_LIFT)] = 50;
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] = 90;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] = 70;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Keyframe 6: Shimmy 2 - 1.3s
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1300;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 105;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 70;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 70;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 100;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_LIFT)] = 90;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Keyframe 7: Ta-da Pose - 1.8s
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1500;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 60;
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 100;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 100;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 70;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 160;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_LIFT)] = 10;
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] = 150;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Keyframe 8: Return to Neutral - 1.8s
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1200;
            auto pos = create_home_pos();
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        m_storage->save_action(silly);
        m_action_cache[silly.name] = silly;
    }
    
    {
        RegisteredAction funny = {};
        strcpy(funny.name, "funny");
        funny.type = ActionType::GAIT_PERIODIC;
        funny.is_atomic = false;
        funny.default_steps = 4;
        funny.data.gait.gait_period_ms = 1500;
        funny.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 30;
        funny.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 10;
        funny.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = -30;
        funny.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 10;
        funny.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = PI;
        funny.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = 5;
        funny.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = 10;
        funny.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = 5;
        funny.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = 10;
        funny.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = PI;
        funny.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_LIFT)] = 5;
        funny.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_LIFT)] = 5;
        funny.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_LIFT)] = PI;
        m_storage->save_action(funny);
        m_action_cache[funny.name] = funny;
    }

    {
        RegisteredAction happy = {};
        strcpy(happy.name, "happy");
        happy.type = ActionType::KEYFRAME_SEQUENCE;
        happy.is_atomic = false;
        happy.default_steps = 1;
        
        auto& kf_data = happy.data.keyframe;
        kf_data.frame_count = 0;

        auto create_home_pos = []() {
            std::array<float, GAIT_JOINT_COUNT> pos;
            for(int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                pos[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
            }
            return pos;
        };

        const float sway_lean = 15.0f;
        const float arm_raise = 60.0f; // Increased arm amplitude
        const float head_pan_amp = 15.0f;
        const float ear_lift_offset = 10.0f; // For forward/back ear movement
        const float ear_swing_offset = 10.0f; // For in/out ear movement

        // Frame 0: Settle into a stable stance
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 600;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] += 10;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] += 10;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // --- Sway Loop (x2) ---
        for (int i = 0; i < 2; ++i) {
            // Frame: Sway Left
            if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
                auto& frame = kf_data.frames[kf_data.frame_count++];
                frame.transition_time_ms = 400; // Faster
                auto pos = create_home_pos();
                // Lean left
                pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] += sway_lean;
                pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] -= sway_lean;
                // Arms gesture
                pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] += arm_raise;
                pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] -= arm_raise / 2;
                // Head pan
                pos[static_cast<int>(ServoChannel::HEAD_PAN)] += head_pan_amp;
                // Ear movements for Sway Left
                pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = ServoCalibration::get_home_pos(ServoChannel::LEFT_EAR_LIFT) - ear_lift_offset; // Back
                pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = ServoCalibration::get_home_pos(ServoChannel::LEFT_EAR_SWING) + ear_swing_offset; // Out
                pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = ServoCalibration::get_home_pos(ServoChannel::RIGHT_EAR_LIFT) + ear_lift_offset; // Forward
                pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = ServoCalibration::get_home_pos(ServoChannel::RIGHT_EAR_SWING) - ear_swing_offset; // In
                memcpy(frame.positions, pos.data(), sizeof(frame.positions));
            }

            // Frame: Sway Right
            if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
                auto& frame = kf_data.frames[kf_data.frame_count++];
                frame.transition_time_ms = 400; // Faster
                auto pos = create_home_pos();
                // Lean right
                pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] -= sway_lean;
                pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] += sway_lean;
                // Arms gesture
                pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] -= arm_raise / 2;
                pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] += arm_raise;
                // Head pan
                pos[static_cast<int>(ServoChannel::HEAD_PAN)] -= head_pan_amp;
                // Ear movements for Sway Right
                pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = ServoCalibration::get_home_pos(ServoChannel::LEFT_EAR_LIFT) + ear_lift_offset; // Forward
                pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = ServoCalibration::get_home_pos(ServoChannel::LEFT_EAR_SWING) - ear_swing_offset; // In
                pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = ServoCalibration::get_home_pos(ServoChannel::RIGHT_EAR_LIFT) - ear_lift_offset; // Back
                pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = ServoCalibration::get_home_pos(ServoChannel::RIGHT_EAR_SWING) + ear_swing_offset; // Out
                memcpy(frame.positions, pos.data(), sizeof(frame.positions));
            }
        }

        // Frame: Return to Center
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 500; // Faster
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] += 10;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] += 10;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = ServoCalibration::get_home_pos(ServoChannel::LEFT_ARM_SWING) + arm_raise;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = ServoCalibration::get_home_pos(ServoChannel::RIGHT_ARM_SWING) + arm_raise;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame: Return to Home
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 600;
            auto pos = create_home_pos();
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        m_storage->save_action(happy);
        m_action_cache[happy.name] = happy;
    }

    {
        RegisteredAction look_around = {};
        strcpy(look_around.name, "Look_Around");
        look_around.type = ActionType::KEYFRAME_SEQUENCE;
        look_around.is_atomic = false;
        look_around.default_steps = 1; // Play sequence only once
        
        auto& kf_data = look_around.data.keyframe;
        kf_data.frame_count = 0;

        auto create_home_pos = []() {
            std::array<float, GAIT_JOINT_COUNT> pos;
            for(int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                pos[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
            }
            return pos;
        };

        // Frame 0: Head bottom-left, ears droop inwards
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 600; // Reduced for faster hand movement
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 75.0f; // Down
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 60.0f;  // Left
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 100.0f; // Forward
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 100.0f;
            // Subtle ears
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 110.0f; // Back
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 60.0f;  // In
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 80.0f;  // Back
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 115.0f; // In
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 1: Head mid, ears perk outwards
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 720; // Reduced for faster hand movement
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 70.0f; // Mid
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 90.0f;  // Mid
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 50.0f; // Backward
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 50.0f;
            // Subtle ears
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 95.0f;   // Front
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 90.0f; // Out
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 100.0f;  // Front
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 80.0f;  // Out
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 2: Head top-right, ears droop outwards
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 660; // Reduced for faster hand movement
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 60.0f; // Up
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 115.0f;  // Right
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 100.0f; // Forward
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 100.0f;
            // Subtle ears
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 110.0f; // Back
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 90.0f; // Out
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 80.0f;  // Back
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 80.0f;  // Out
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 3: Head mid, ears perk outwards (same as frame 1)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 720; // Reduced for faster hand movement
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 70.0f; // Mid
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 90.0f;  // Mid
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 50.0f; // Backward
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 50.0f;
            // Subtle ears
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 95.0f;   // Front
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 90.0f; // Out
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 100.0f;  // Front
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 80.0f;  // Out
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 4: Head bottom-left, ears droop inwards (same as frame 0)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 600; // Reduced for faster hand movement
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 75.0f; // Down
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 60.0f;  // Left
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 100.0f; // Forward
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 100.0f;
            // Subtle ears
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 110.0f; // Back
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 60.0f;  // In
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 80.0f;  // Back
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 115.0f; // In
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 5: Return to home
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1000;
            auto pos = create_home_pos();
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        m_storage->save_action(look_around);
        m_action_cache[look_around.name] = look_around;
    }

    {
        RegisteredAction very_happy = {};
        strcpy(very_happy.name, "very_happy"); // Replacing laughing
        very_happy.type = ActionType::KEYFRAME_SEQUENCE;
        very_happy.is_atomic = false;
        very_happy.default_steps = 1;
        
        auto& kf_data = very_happy.data.keyframe;
        kf_data.frame_count = 0;

        auto create_home_pos = []() {
            std::array<float, GAIT_JOINT_COUNT> pos;
            for(int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                pos[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
            }
            return pos;
        };

        // --- Part 1: Symmetrical Dance ---

        // Frame 0: Arms up, body crouch
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 800;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 130; // up
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 130; // up
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] = 100; // crouch
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] = 110; // crouch
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 95; // slightly forward
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 95; // slightly forward
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 75; // slightly in
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 110; // slightly in
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 1: Arms down, stand up
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 800;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 60; // down
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 60; // down
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] = 80; // stand
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] = 95; // stand
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 105; // slightly back
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 85; // slightly back
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 85; // slightly out
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 100; // slightly out
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 2: Arms in, body twist left
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 800;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 110; // in
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_LIFT)] = 110; // in
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] = 45;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] = 125;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 90; // forward
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 100; // forward
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 70; // in
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 120; // in
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 3: Arms out, body twist right
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 800;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 30; // out
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_LIFT)] = 30; // out
            pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] = 125;
            pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] = 45;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 110; // back
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 80; // back
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 90; // out
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 90; // out
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 4: Transition to home
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1000; // Slow transition to home
            auto pos = create_home_pos();
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // --- Part 2: Replace with walk_forward_kf logic (first 15 frames) ---
        const int frame_time = 1200 / 16; 

        // Generate and add frames from walk_forward_kf
        for (int i = 0; i < 15; ++i) { // Add 15 frames to reach the 20-frame limit
            if (kf_data.frame_count >= MAX_KEYFRAMES_PER_ACTION) break;

            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = frame_time;
            auto pos = create_home_pos();
            float theta = (float)i * 2.0f * PI / 16.0f;

            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 78.0f;
            if (i < 8) { // First half of the cycle
                theta = (float)i * 2.0f * PI / 16.0f;
                pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] += 33.0f * sin(theta);
                pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] += 33.0f * sin(theta);
                pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] += 15.0f * cos(theta);
                pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] += 15.0f * cos(theta);
                pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] -= 50.0f * sin(theta);
                pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] += 50.0f * sin(theta);
            } else { // Second half of the cycle
                theta = (float)(i-8) * 2.0f * PI / 16.0f;
                pos[static_cast<int>(ServoChannel::LEFT_LEG_ROTATE)] -= 33.0f * sin(theta);
                pos[static_cast<int>(ServoChannel::RIGHT_LEG_ROTATE)] -= 33.0f * sin(theta);
                pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] -= 15.0f * cos(theta);
                pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] -= 15.0f * cos(theta);
                pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] += 50.0f * sin(theta);
                pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] -= 50.0f * sin(theta);
            }
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        m_storage->save_action(very_happy);
        m_action_cache[very_happy.name] = very_happy;
    }

    {
        RegisteredAction angry_head = {};
        strcpy(angry_head.name, "angry_head");
        angry_head.type = ActionType::KEYFRAME_SEQUENCE;
        angry_head.is_atomic = true;
        angry_head.default_steps = 1;
        
        auto& kf_data = angry_head.data.keyframe;
        kf_data.frame_count = 0;

        auto create_home_pos = []() {
            std::array<float, GAIT_JOINT_COUNT> pos;
            for(int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                pos[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
            }
            return pos;
        };

        // Frame 0: Initial Pose - Head down, arms set (100ms)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 100;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 75.0f; // Look down (less pronounced)
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 80.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 100.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 70.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 50.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 1: Head moves left (500ms)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 500;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 75.0f; // Keep down
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 10.0f;  // Full left
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 80.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 100.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 70.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 50.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 2: Head holds (500ms)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 500;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 75.0f; // Keep down
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 10.0f;  // Hold left
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 80.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 100.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 70.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 50.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 3: "Roll eyes", Head lifts (200ms)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 200;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 60.0f; // Head lifts
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 27.5f;  // Halfway center
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 85.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 95.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 80.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 70.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 4: Return to bottom-left (200ms)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 200;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 75.0f; // Down
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 10.0f;  // Full left
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 80.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 100.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 70.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 50.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 5 & 6: Hold pose (1500ms total)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1500;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 75.0f; // Hold
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 10.0f;  // Hold
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 80.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 100.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 70.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 50.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 7: Return to Home (500ms)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 500;
            auto pos = create_home_pos();
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        m_storage->save_action(angry_head);
        m_action_cache[angry_head.name] = angry_head;
    }

    {
        RegisteredAction stomp_left_foot = {};
        strcpy(stomp_left_foot.name, "stomp_left_foot");
        stomp_left_foot.type = ActionType::GAIT_PERIODIC;
        stomp_left_foot.is_atomic = false;
        stomp_left_foot.default_steps = 4; // Run for 4 cycles
        stomp_left_foot.data.gait.gait_period_ms = 1000; // 1 second per stomp cycle
        // Use offset to create a two-part motion within the sine wave
        // Part 1: Lift and kick forward (controlled by offset and amplitude)
        stomp_left_foot.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 92.5f - 100.0f; // Center of motion is 92.5, home is 100
        stomp_left_foot.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 27.5f; // Goes from 65 to 120
        // Part 2: Phase shift to make ANKLE_LIFT lead the ROTATE, creating a circular path
        stomp_left_foot.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = PI / 2;

        m_storage->save_action(stomp_left_foot);
        m_action_cache[stomp_left_foot.name] = stomp_left_foot;
    }

    {
        RegisteredGroup angry_group = {};
        strcpy(angry_group.name, "angry");
        angry_group.action_count = 2;
        strcpy(angry_group.action_names[0], "angry_head");
        strcpy(angry_group.action_names[1], "stomp_left_foot");
        m_storage->save_group(angry_group);
        m_group_cache[angry_group.name] = angry_group;
    }

    {
        RegisteredAction sudden_shock = {};
        strcpy(sudden_shock.name, "sudden_shock");
        sudden_shock.type = ActionType::KEYFRAME_SEQUENCE;
        sudden_shock.is_atomic = true; // This is a fast, atomic reaction
        sudden_shock.default_steps = 1;
        
        auto& kf_data = sudden_shock.data.keyframe;
        kf_data.frame_count = 0;

        auto create_home_pos = []() {
            std::array<float, GAIT_JOINT_COUNT> pos;
            for(int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                pos[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
            }
            return pos;
        };

        // Frame 0: The Jolt - Head up, Ears out
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 350;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] = 100.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] = 110.0f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 80.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_LIFT)] = 80.0f;
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 60.0f; // Head jerks up (less than before)
            // Ears fly up and out
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 60.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 120.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 1: Ear Recoil - Ears snap in and down
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 350;
            auto pos = create_home_pos();
            // Body and head hold the jolted pose
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] = 100.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] = 110.0f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 80.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_LIFT)] = 80.0f;
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 65.0f;
            // Ears snap in and down
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 60.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 120.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 2: The Freeze (holding the recoil pose)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1000; // Shortened freeze
            auto pos = create_home_pos();
            // Hold the recoil pose from frame 1
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] = 100.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ANKLE_LIFT)] = 110.0f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 80.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_LIFT)] = 80.0f;
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 65.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 60.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 120.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 3: Slow Head Scan Left
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1400;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 100.0f;
            // Keep shocked expression
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 65.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 60.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 120.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 4: Slow Head Scan Right
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1400;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 40.0f;
            // Keep shocked expression
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 65.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 60.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 120.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 5: Full Return to Home
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1000;
            auto pos = create_home_pos();
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        m_storage->save_action(sudden_shock);
        m_action_cache[sudden_shock.name] = sudden_shock;
    }

    {
        RegisteredAction curious_ponder = {};
        strcpy(curious_ponder.name, "curious_ponder");
        curious_ponder.type = ActionType::KEYFRAME_SEQUENCE;
        curious_ponder.is_atomic = false;
        curious_ponder.default_steps = 1;
        
        auto& kf_data = curious_ponder.data.keyframe;
        kf_data.frame_count = 0;

        auto create_home_pos = []() {
            std::array<float, GAIT_JOINT_COUNT> pos;
            for(int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                pos[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
            }
            return pos;
        };

        // Frame 0: Slight nod down
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 500;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 80.0f; // Slightly down from home (70)
            // Ears perk up slightly
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 95.0f; // Slightly forward from home (100)
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 95.0f; // Slightly forward from home (90)
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 1: Slow turn left
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1000;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 80.0f; // Keep looking slightly down
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 130.0f; // Turn left (Home 90, Left is higher)
            // Ears orient towards the sound
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 95.0f; // Left ear out
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 80.0f; // Right ear slightly out
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 2: Hold and observe
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1500; // Hold for 1.5 seconds
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 80.0f;
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 130.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 95.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 80.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 3: Return to Home
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1000;
            auto pos = create_home_pos(); // All servos return to calibrated home
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        m_storage->save_action(curious_ponder);
        m_action_cache[curious_ponder.name] = curious_ponder;
    }

    {
        RegisteredAction sad = {};
        strcpy(sad.name, "sad");
        sad.type = ActionType::KEYFRAME_SEQUENCE;
        sad.is_atomic = false;
        sad.default_steps = 1;
        
        auto& kf_data = sad.data.keyframe;
        kf_data.frame_count = 0;

        auto create_home_pos = []() {
            std::array<float, GAIT_JOINT_COUNT> pos;
            for(int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                pos[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
            }
            return pos;
        };

        // Frame 0: Droop
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1500;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 80.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 120.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 120.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 60.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 120.0f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 80.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 100.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 1: Head Shake "No" Left
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1000;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 80.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 120.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 120.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 60.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 120.0f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 80.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 100.0f;
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 60.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 2: Head Shake "No" Right
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1000;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 80.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 120.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 120.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 60.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 120.0f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 80.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 100.0f;
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 120.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 3: Look down further
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1000;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 80.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 120.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 120.0f;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_SWING)] = 60.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_SWING)] = 120.0f;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 80.0f;
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 100.0f;
            pos[static_cast<int>(ServoChannel::HEAD_PAN)] = 90.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 4: Return to home
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 2000;
            auto pos = create_home_pos();
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        m_storage->save_action(sad);
        m_action_cache[sad.name] = sad;
    }

    {
        RegisteredAction tracking_L = {};
        strcpy(tracking_L.name, "tracking_L");
        tracking_L.type = ActionType::GAIT_PERIODIC;
        tracking_L.default_steps = 1;
        tracking_L.is_atomic = false;
        tracking_L.data.gait.gait_period_ms = 1500;
        tracking_L.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = 40;
        tracking_L.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -40;
        tracking_L.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -30;
        tracking_L.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 35;
        tracking_L.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = PI / 2 + PI;
        tracking_L.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = PI;
        tracking_L.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = PI;
        tracking_L.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 2 * PI;
        m_storage->save_action(tracking_L); 
        m_action_cache[tracking_L.name] = tracking_L;
    }

    {
        RegisteredAction tracking_R = m_action_cache["tracking_L"];
        strcpy(tracking_R.name, "tracking_R");
        tracking_R.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -40;
        tracking_R.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 40;
        tracking_R.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 35;
        tracking_R.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = -35;
        tracking_R.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 0;
        tracking_R.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = PI / 2;
        tracking_R.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 0;
        m_storage->save_action(tracking_R);
        m_action_cache[tracking_R.name] = tracking_R;
    }

    { // Scope for wave_hello - V3, faster wave
        RegisteredAction wave_hello = {};
        strcpy(wave_hello.name, "wave_hello");
        wave_hello.type = ActionType::KEYFRAME_SEQUENCE;
        wave_hello.is_atomic = false;
        wave_hello.default_steps = 1;
        
        auto& kf_data = wave_hello.data.keyframe;
        kf_data.frame_count = 0;

        auto create_home_pos = []() {
            std::array<float, GAIT_JOINT_COUNT> pos;
            for(int i = 0; i < GAIT_JOINT_COUNT; ++i) {
                pos[i] = ServoCalibration::get_home_pos(static_cast<ServoChannel>(i));
            }
            return pos;
        };

        // Frame 1: Raise arm and tilt head up (1.0s)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1000;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 145;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 70;
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 60;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 95;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 100;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 2: Wave In (0.2s)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 400; // FASTER
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 145;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 100;
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 60;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 110;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 80;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 3: Wave Out (0.2s)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 400; // FASTER
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 145;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 70;
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 60;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 110;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 80;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 4: Wave In (0.2s)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 400; // FASTER
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 145;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 100;
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 60;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 95;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 100;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }
        
        // Frame 5: Wave Out (0.2s)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 400; // FASTER
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::LEFT_ARM_SWING)] = 145;
            pos[static_cast<int>(ServoChannel::LEFT_ARM_LIFT)] = 70;
            pos[static_cast<int>(ServoChannel::HEAD_TILT)] = 60;
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 95;
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 100;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 6: Lower arm (1.0s)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 1000;
            auto pos = create_home_pos(); // Return all to home
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        m_storage->save_action(wave_hello);
        m_action_cache[wave_hello.name] = wave_hello;
    }

    ESP_LOGI(TAG, "Default actions created and cached.");
}

bool ActionManager::delete_action_from_nvs(const std::string& action_name) {
    ESP_LOGI(TAG, "Attempting to delete action '%s' from NVS...", action_name.c_str());
    bool success = m_storage->delete_action(action_name.c_str());
    if (success) {
        if (m_action_cache.count(action_name)) {
            m_action_cache.erase(action_name);
            ESP_LOGI(TAG, "Action '%s' removed from cache.", action_name.c_str());
        }
    }
    return success;
}

bool ActionManager::delete_group_from_nvs(const std::string& group_name) {
    ESP_LOGI(TAG, "Attempting to delete group '%s' from NVS...", group_name.c_str());
    bool success = m_storage->delete_group(group_name.c_str());
    if (success) {
        if (m_group_cache.count(group_name)) {
            m_group_cache.erase(group_name);
            ESP_LOGI(TAG, "Group '%s' removed from cache.", group_name.c_str());
        }
    }
    return success;
}

std::vector<std::string> ActionManager::list_actions_from_nvs() {
    std::vector<std::string> actions;
    m_storage->list_actions(actions);
    ESP_LOGI(TAG, "Found %d actions in NVS.", actions.size());
    return actions;
}

std::vector<std::string> ActionManager::list_groups_from_nvs() {
    std::vector<std::string> groups;
    m_storage->list_groups(groups);
    ESP_LOGI(TAG, "Found %d groups in NVS.", groups.size());
    return groups;
}

bool ActionManager::update_action_properties(const std::string& action_name, bool is_atomic, uint32_t default_steps, uint32_t gait_period_ms) {
    auto it = m_action_cache.find(action_name);
    if (it == m_action_cache.end()) {
        ESP_LOGE(TAG, "Action '%s' not found in cache for property update.", action_name.c_str());
        return false;
    }
    RegisteredAction& action = it->second;
    action.is_atomic = is_atomic;
    action.default_steps = default_steps;
    if (action.type == ActionType::GAIT_PERIODIC) {
        action.data.gait.gait_period_ms = gait_period_ms;
    }
    ESP_LOGI(TAG, "Updated properties for action '%s': is_atomic=%s, steps=%d", 
             action_name.c_str(), is_atomic ? "true" : "false", (int)default_steps);
    print_action_details(action);
    return true;
}

bool ActionManager::tune_gait_parameter(const std::string& action_name, int servo_index, const std::string& param_type, float value) {
    auto it = m_action_cache.find(action_name);
    if (it == m_action_cache.end()) {
        ESP_LOGE(TAG, "Action '%s' not found in cache for tuning.", action_name.c_str());
        return false;
    }
    if (servo_index < 0 || servo_index >= GAIT_JOINT_COUNT) {
        ESP_LOGE(TAG, "Invalid servo index: %d", servo_index);
        return false;
    }
    RegisteredAction& action = it->second;
    if (action.type != ActionType::GAIT_PERIODIC) {
        ESP_LOGE(TAG, "Cannot tune non-gait action '%s'", action_name.c_str());
        return false;
    }
    if (param_type == "amplitude") action.data.gait.params.amplitude[servo_index] = value;
    else if (param_type == "offset") action.data.gait.params.offset[servo_index] = value;
    else if (param_type == "phase_diff") action.data.gait.params.phase_diff[servo_index] = value;
    else {
        ESP_LOGE(TAG, "Unknown parameter type: %s", param_type.c_str());
        return false;
    }
    ESP_LOGI(TAG, "Tuned %s for %s, servo %d: set to %.2f", param_type.c_str(), action_name.c_str(), servo_index, value);
    return true;
}

bool ActionManager::save_action_to_nvs(const std::string& action_name) {
    auto it = m_action_cache.find(action_name);
    if (it == m_action_cache.end()) {  
        ESP_LOGE(TAG, "Action '%s' not found in cache, cannot save.", action_name.c_str());
        return false;
    }
    ESP_LOGI(TAG, "Saving action '%s' to NVS...", action_name.c_str());
    return m_storage->save_action(it->second);
}

std::string ActionManager::get_action_params_json(const std::string& action_name) {
    auto it = m_action_cache.find(action_name);
    if (it == m_action_cache.end()) return "{}";
  
    const RegisteredAction& action = it->second;
    char temp_buf[256];

    if (action.type == ActionType::GAIT_PERIODIC) {
        std::string json_str;
        json_str.reserve(1024);
        snprintf(temp_buf, sizeof(temp_buf), 
            "{\"name\":\"%s\",\"type\":\"gait\",\"is_atomic\":%s,\"default_steps\":%d,\"gait_period_ms\":%d,\"params\":{", 
            action.name, action.is_atomic ? "true" : "false", (int)action.default_steps, (int)action.data.gait.gait_period_ms);
        json_str += temp_buf;

        auto& params = action.data.gait.params;
        json_str += "\"amplitude\":[";
        for(int i=0; i<GAIT_JOINT_COUNT; ++i) {
            snprintf(temp_buf, sizeof(temp_buf), "%.2f", params.amplitude[i]);
            json_str += temp_buf;
            if (i < GAIT_JOINT_COUNT - 1) json_str += ",";
        }
        json_str += "],";
        json_str += "\"offset\":[";
        for(int i=0; i<GAIT_JOINT_COUNT; ++i) {
            snprintf(temp_buf, sizeof(temp_buf), "%.2f", params.offset[i]);
            json_str += temp_buf;
            if (i < GAIT_JOINT_COUNT - 1) json_str += ",";
        }
        json_str += "],";
        json_str += "\"phase_diff\":[";
        for(int i=0; i<GAIT_JOINT_COUNT; ++i) {
            snprintf(temp_buf, sizeof(temp_buf), "%.2f", params.phase_diff[i]);
            json_str += temp_buf;
            if (i < GAIT_JOINT_COUNT - 1) json_str += ",";
        }
        json_str += "]}}";
        return json_str;
    } else if (action.type == ActionType::KEYFRAME_SEQUENCE) {
        snprintf(temp_buf, sizeof(temp_buf),
            "{\"name\":\"%s\",\"type\":\"keyframe\",\"is_atomic\":%s,\"default_steps\":%d,\"frame_count\":%d}",
            action.name, action.is_atomic ? "true" : "false", (int)action.default_steps, (int)action.data.keyframe.frame_count);
        return std::string(temp_buf);
    }
    return "{}";
}

void ActionManager::print_action_details(const RegisteredAction &action) {
    ESP_LOGI(TAG, "[Action Details] Name: %s", action.name);
    if (action.type == ActionType::GAIT_PERIODIC) {
        ESP_LOGI(TAG, "  - Type: Gait Periodic");
        ESP_LOGI(TAG, "  - Atomic: %s", action.is_atomic ? "Yes" : "No");
        ESP_LOGI(TAG, "  - Steps: %d", (int)action.default_steps);
        ESP_LOGI(TAG, "  - Period: %d ms", (int)action.data.gait.gait_period_ms);
        
        auto print_float_array = [](const char* prefix, const float* arr) {
            char buffer[256];
            int offset = snprintf(buffer, sizeof(buffer), "  - %s: [", prefix);
            for(int i=0; i <GAIT_JOINT_COUNT; ++i) {
                offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%.2f, ", arr[i]);
            }
            if (offset > 2) { 
                buffer[offset-2] = ']';
                buffer[offset-1] = '\0';
            }
            ESP_LOGI(TAG, "%s", buffer);
        };

        print_float_array("Amplitude", action.data.gait.params.amplitude);
        print_float_array("Offset   ", action.data.gait.params.offset);
        print_float_array("Phase    ", action.data.gait.params.phase_diff);

    } else if (action.type == ActionType::KEYFRAME_SEQUENCE) {
        ESP_LOGI(TAG, "  - Type: Keyframe Sequence");
        ESP_LOGI(TAG, "  - Atomic: %s", action.is_atomic ? "Yes" : "No");
        ESP_LOGI(TAG, "  - Steps: %d", (int)action.default_steps);
        ESP_LOGI(TAG, "  - Frame Count: %d", (int)action.data.keyframe.frame_count);
        for (int i = 0; i < action.data.keyframe.frame_count; ++i) {
            ESP_LOGI(TAG, "    - Frame %d: transition_time=%dms", i, action.data.keyframe.frames[i].transition_time_ms);
        }
    } else {
        ESP_LOGI(TAG, "  - Type: Unknown");
    }
}