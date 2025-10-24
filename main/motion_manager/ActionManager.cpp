#include "motion_manager/ActionManager.hpp"
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

void ActionManager::register_default_actions(bool force) {
    ESP_LOGI(TAG, "Checking and registering default actions...");

    if (!force) {
        RegisteredAction temp_action, temp_wave_action;
        if (m_storage->load_action("walk_forward", temp_action) && m_storage->load_action("wave_hello", temp_wave_action)) {
            ESP_LOGI(TAG, "All default actions (including new ones) found in NVS. Loading from storage.");
            m_action_cache["walk_forward"] = temp_action;
            m_storage->load_action("walk_backward", m_action_cache["walk_backward"]);
            m_storage->load_action("turn_left", m_action_cache["turn_left"]);
            m_storage->load_action("turn_right", m_action_cache["turn_right"]);
            m_storage->load_action("wiggle_ears", m_action_cache["wiggle_ears"]);
            m_storage->load_action("wave_hand", m_action_cache["wave_hand"]);
            m_storage->load_action("nod_head", m_action_cache["nod_head"]);
            m_storage->load_action("shake_head", m_action_cache["shake_head"]);
            m_storage->load_action("single_leg", m_action_cache["single_leg"]);
            m_storage->load_action("happy", m_action_cache["happy"]);
            m_storage->load_action("sad", m_action_cache["sad"]);
            m_storage->load_action("silly", m_action_cache["silly"]);
            m_storage->load_action("funny", m_action_cache["funny"]);
            m_storage->load_action("laughing", m_action_cache["laughing"]);
            m_storage->load_action("angry", m_action_cache["angry"]);
            m_storage->load_action("crying", m_action_cache["crying"]);
            m_storage->load_action("surprised", m_action_cache["surprised"]);
            m_storage->load_action("thinking", m_action_cache["thinking"]);
            m_storage->load_action("tracking_L", m_action_cache["tracking_L"]);
            m_storage->load_action("tracking_R", m_action_cache["tracking_R"]);
            m_action_cache["wave_hello"] = temp_wave_action; // Also load the new action
            return;
        }
    }

    ESP_LOGI(TAG, "Default actions not found or outdated. Creating/re-creating them...");

    { // Scope for forward
        RegisteredAction forward = {};
        strcpy(forward.name, "walk_forward");
        forward.type = ActionType::GAIT_PERIODIC;
        forward.is_atomic = false;
        forward.default_steps = 4;
        forward.data.gait.gait_period_ms = 1500;
        forward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = 33;
        forward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 33;
        forward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = 25;
        forward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]  = 25;
        forward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)]  = 60;
        forward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)]  = 60;
        forward.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -20;
        forward.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 10;
        forward.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = PI / 2;
        forward.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]  = PI / 2;
        forward.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]   = 0;
        forward.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)]  = 0;
        m_storage->save_action(forward);
        m_action_cache[forward.name] = forward;
    }

    { // Scope for backward
        RegisteredAction backward = m_action_cache["walk_forward"]; // Copy from cache
        strcpy(backward.name, "walk_backward");
        backward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -33;
        backward.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -33;
        m_storage->save_action(backward);
        m_action_cache[backward.name] = backward;
    }

    { // Scope for left
        RegisteredAction left = m_action_cache["walk_forward"];
        strcpy(left.name, "turn_left");
        left.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -25;
        left.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 45;
        left.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
        left.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = -10;
        left.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]   = PI / 2;
        m_storage->save_action(left);
        m_action_cache[left.name] = left;
    }

    { // Scope for right
        RegisteredAction right = m_action_cache["turn_left"];
        strcpy(right.name, "turn_right");
        right.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = 25;
        right.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -25;
        right.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -20;
        right.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -15;
        right.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 0;
        right.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = PI / 2;
        m_storage->save_action(right);
        m_action_cache[right.name] = right;
    }

    { // Scope for wiggle_ears
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
        wave_hand.type = ActionType::GAIT_PERIODIC;
        wave_hand.is_atomic = false;
        wave_hand.default_steps = 2;
        wave_hand.data.gait.gait_period_ms = 1500;
        wave_hand.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)] = 30;
        wave_hand.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = 30;
        wave_hand.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = 20;
        wave_hand.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = 20;
        m_storage->save_action(wave_hand);
        m_action_cache[wave_hand.name] = wave_hand;
    }

    { // Scope for nod_head
        RegisteredAction nod_head = {};
        strcpy(nod_head.name, "nod_head");
        nod_head.type = ActionType::GAIT_PERIODIC;
        nod_head.is_atomic = false;
        nod_head.default_steps = 2;
        nod_head.data.gait.gait_period_ms = 1500;
        nod_head.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 20;
        m_storage->save_action(nod_head);
        m_action_cache[nod_head.name] = nod_head;
    }

    { // Scope for shake_head
        RegisteredAction shake_head = m_action_cache["nod_head"];
        strcpy(shake_head.name, "shake_head");
        shake_head.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = 20;
        shake_head.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 0;
        m_storage->save_action(shake_head);
        m_action_cache[shake_head.name] = shake_head;
    }

    { // Scope for single_leg
        RegisteredAction single_leg = {};
        strcpy(single_leg.name, "single_leg");
        single_leg.type = ActionType::GAIT_PERIODIC;
        single_leg.is_atomic = false;
        single_leg.default_steps = 2;
        single_leg.data.gait.gait_period_ms = 1500;
        single_leg.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 1;
        single_leg.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 1; 
        single_leg.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 30;
        single_leg.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 30;
        m_storage->save_action(single_leg);
        m_action_cache[single_leg.name] = single_leg;
    }

    { // Scope for silly
        RegisteredAction silly = {};
        strcpy(silly.name, "silly");
        silly.type = ActionType::GAIT_PERIODIC;
        silly.is_atomic = false;       
        silly.default_steps = 4;            
        silly.data.gait.gait_period_ms = 2000;        
        silly.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 30;
        silly.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 30;
        silly.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 40;
        silly.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = -40;
        silly.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -15;
        silly.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 15;
        silly.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = PI;
        m_storage->save_action(silly);
        m_action_cache[silly.name] = silly;
    }
    
    { // Scope for funny
        RegisteredAction funny = {};
        strcpy(funny.name, "funny");
        funny.type = ActionType::GAIT_PERIODIC;
        funny.is_atomic = false;
        funny.default_steps = 3;
        funny.data.gait.gait_period_ms = 1200;
        funny.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -50;
        funny.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = -30;
        funny.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 15;
        funny.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 20;
        funny.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 20;
        m_storage->save_action(funny);
        m_action_cache[funny.name] = funny;
    }

    { // Scope for happy
        RegisteredAction happy = {};
        strcpy(happy.name, "happy");
        happy.type = ActionType::GAIT_PERIODIC;
        happy.is_atomic = false;
        happy.default_steps = 8;
        happy.data.gait.gait_period_ms = 800;
        happy.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
        happy.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
        happy.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 15;
        happy.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 15;
        happy.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = PI;
        happy.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = 25;
        happy.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 10;
        happy.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = PI / 2;
        happy.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 30;
        happy.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 30;
        happy.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = -45;
        happy.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = -45;
        happy.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)] = 40;
        happy.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = 40;
        happy.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = PI;
        m_storage->save_action(happy);
        m_action_cache[happy.name] = happy;
    }

    { // Scope for sad
        RegisteredAction sad = {};
        strcpy(sad.name, "sad");
        sad.type = ActionType::GAIT_PERIODIC;
        sad.is_atomic = false;
        sad.default_steps = 4;
        sad.data.gait.gait_period_ms = 2200;
        sad.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
        sad.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
        sad.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 15;
        sad.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 15;
        sad.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = PI;
        sad.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 30;
        sad.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = 10;
        sad.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_EAR_LIFT)] = 20;
        sad.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_LIFT)] = 20;
        sad.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = -50;
        sad.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = -50;
        sad.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)] = 30;
        sad.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = -30;
        sad.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = 5;
        sad.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = 5;
        sad.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = 0;
        m_storage->save_action(sad);
        m_action_cache[sad.name] = sad;
    }

    { // Scope for laughing
        RegisteredAction laughing = {};
        strcpy(laughing.name, "laughing");
        laughing.type = ActionType::GAIT_PERIODIC;
        laughing.is_atomic = false;
        laughing.default_steps = 6;
        laughing.data.gait.gait_period_ms = 1000;
        laughing.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = -25;
        laughing.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = -70;
        laughing.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 10;
        laughing.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -20;
        laughing.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 40;
        laughing.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = -40;
        laughing.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = 10;
        laughing.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 15;
        laughing.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 15;
        laughing.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = PI/2;
        m_storage->save_action(laughing);
        m_action_cache[laughing.name] = laughing;
    }

    { // Scope for angry
        RegisteredAction angry = {};
        strcpy(angry.name, "angry");
        angry.type = ActionType::GAIT_PERIODIC;
        angry.is_atomic = true;
        angry.default_steps = 4;
        angry.data.gait.gait_period_ms = 1000;
        angry.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = 10;
        angry.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = 10;
        angry.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 20;
        angry.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 20;
        angry.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = PI;
        angry.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = -30;
        angry.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 30;
        angry.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 20;
        angry.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = 20;
        angry.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = -20;
        angry.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 10;
        m_storage->save_action(angry);
        m_action_cache[angry.name] = angry;
    }

    { // Scope for crying
        RegisteredAction crying = {};
        strcpy(crying.name, "crying");
        crying.type = ActionType::GAIT_PERIODIC;
        crying.is_atomic = false;
        crying.default_steps = 5;
        crying.data.gait.gait_period_ms = 2500;
        crying.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
        crying.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
        crying.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 8;
        crying.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 8;
        crying.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = PI;
        crying.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 40;
        crying.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = -60;
        crying.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = -60;
        crying.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)] = 20;
        crying.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = 20;
        crying.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = PI;
        m_storage->save_action(crying);
        m_action_cache[crying.name] = crying;
    }

    { // Scope for surprised
        RegisteredAction surprised = {};
        strcpy(surprised.name, "surprised");
        surprised.type = ActionType::GAIT_PERIODIC;
        surprised.is_atomic = false;
        surprised.default_steps = 1;      
        surprised.data.gait.gait_period_ms = 1200; 
        surprised.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
        surprised.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
        surprised.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = -15;
        surprised.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -15;
        surprised.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = -20;
        surprised.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_EAR_LIFT)] = -30;
        surprised.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_LIFT)] = -30;
        surprised.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = -30;
        surprised.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = -30;
        surprised.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)] = -20;
        surprised.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = 20;
        m_storage->save_action(surprised);
        m_action_cache[surprised.name] = surprised;
    }

    { // Scope for thinking
        RegisteredAction thinking = {};
        strcpy(thinking.name, "thinking");
        thinking.type = ActionType::GAIT_PERIODIC;
        thinking.is_atomic = false;
        thinking.default_steps = 4;
        thinking.data.gait.gait_period_ms = 3000; 
        thinking.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
        thinking.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
        thinking.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 15;
        thinking.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 15;
        thinking.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = 25;
        thinking.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = 5;
        thinking.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 30;
        thinking.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 5;
        thinking.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 5;
        thinking.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 5;
        thinking.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = -70; 
        thinking.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)] = 20;  
        m_storage->save_action(thinking);
        m_action_cache[thinking.name] = thinking;
    }
    
    { // Scope for tracking_L
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
        tracking_L.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 20;
        tracking_L.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 20;
        tracking_L.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 12;
        tracking_L.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 12;
        tracking_L.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = PI / 2 + PI;
        tracking_L.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = PI;
        tracking_L.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = PI;
        tracking_L.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 2 * PI;
        m_storage->save_action(tracking_L); 
        m_action_cache[tracking_L.name] = tracking_L;
    }

    { // Scope for tracking_R
        RegisteredAction tracking_R = m_action_cache["tracking_L"];
        strcpy(tracking_R.name, "tracking_R");
        tracking_R.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -40;
        tracking_R.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 40;
        tracking_R.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 35;
        tracking_R.data.gait.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = -35;
        tracking_R.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 0;
        tracking_R.data.gait.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 0;
        tracking_R.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 0;
        tracking_R.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = PI / 2;
        tracking_R.data.gait.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 0;
        m_storage->save_action(tracking_R);
        m_action_cache[tracking_R.name] = tracking_R;
    }

    { // Scope for wave_hello (KEYFRAME)
        RegisteredAction wave_hello = {};
        strcpy(wave_hello.name, "wave_hello");
        wave_hello.type = ActionType::KEYFRAME_SEQUENCE;
        wave_hello.is_atomic = false;
        wave_hello.default_steps = 4; // Execute 4 times
        
        auto& kf_data = wave_hello.data.keyframe;
        kf_data.frame_count = 0;

        auto create_home_pos = []() {
            std::array<float, GAIT_JOINT_COUNT> pos;
            for(int i = 0; i < GAIT_JOINT_COUNT; ++i) pos[i] = 90.0f;
            return pos;
        };

        // Frame 0: Get into position (500ms)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 500;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 0.0f;   // Arm up to 0 degrees
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] = 30.0f;  // Left foot tiptoe
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 110.0f; // Ears start drooping
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 110.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 1: Wave "back" (250ms - Fast)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 250;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 20.0f;  // Wave back
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] = 30.0f;  // Hold tiptoe
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 120.0f; // Ears continue drooping
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 120.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 2: Wave "forward" (250ms - Fast)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 250;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 0.0f;   // Wave forward
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] = 30.0f;  // Hold tiptoe
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 120.0f; // Ears reach full droop
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 120.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 3: Wave "back" (250ms - Fast)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 250;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 20.0f;  // Wave back again
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] = 30.0f;  // Hold tiptoe
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 120.0f; // Hold droop
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 120.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 4: Wave "forward" (250ms - Fast)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 250;
            auto pos = create_home_pos();
            pos[static_cast<int>(ServoChannel::RIGHT_ARM_SWING)] = 0.0f;   // Wave forward again
            pos[static_cast<int>(ServoChannel::LEFT_ANKLE_LIFT)] = 30.0f;  // Hold tiptoe
            pos[static_cast<int>(ServoChannel::LEFT_EAR_LIFT)] = 120.0f; // Hold droop
            pos[static_cast<int>(ServoChannel::RIGHT_EAR_LIFT)] = 120.0f;
            memcpy(frame.positions, pos.data(), sizeof(frame.positions));
        }

        // Frame 5: Return to Home (600ms)
        if (kf_data.frame_count < MAX_KEYFRAMES_PER_ACTION) {
            auto& frame = kf_data.frames[kf_data.frame_count++];
            frame.transition_time_ms = 600;
            auto pos = create_home_pos(); // All servos return to 90
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
            for(int i=0; i<GAIT_JOINT_COUNT; ++i) {
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
  