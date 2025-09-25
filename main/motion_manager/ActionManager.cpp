#include "motion_manager/ActionManager.hpp"
#include "esp_log.h"
#include <cmath>
#include <cstring>

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
    register_default_actions();
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

void ActionManager::register_default_actions() {
    ESP_LOGI(TAG, "Checking and registering default actions...");

    RegisteredAction temp_action;
    if (m_storage->load_action("walk_forward", temp_action)) {
        ESP_LOGI(TAG, "Default actions found in NVS. Loading from storage.");
        m_action_cache["walk_forward"] = temp_action;
        m_storage->load_action("walk_backward", m_action_cache["walk_backward"]);
        m_storage->load_action("turn_left", m_action_cache["turn_left"]);
        m_storage->load_action("turn_right", m_action_cache["turn_right"]);
        m_storage->load_action("wiggle_ears", m_action_cache["wiggle_ears"]);
        m_storage->load_action("wave_hand", m_action_cache["wave_hand"]);
        m_storage->load_action("nod_head", m_action_cache["nod_head"]);
        m_storage->load_action("shake_head", m_action_cache["shake_head"]);
        m_storage->load_action("single_leg", m_action_cache["single_leg"]);
        m_storage->load_action("tracking_L", m_action_cache["tracking_L"]);
        m_storage->load_action("tracking_R", m_action_cache["tracking_R"]);
        m_storage->load_action("dance", m_action_cache["dance"]);
        m_storage->load_action("leg_swing", m_action_cache["leg_swing"]);
        m_storage->load_action("happy_wiggle", m_action_cache["happy_wiggle"]);
        m_storage->load_action("sad_stomp", m_action_cache["sad_stomp"]);
        return;
    }

    ESP_LOGI(TAG, "Default actions not found in NVS. Creating them...");

    RegisteredAction forward = {};
    strcpy(forward.name, "walk_forward");
    forward.type = ActionType::GAIT_PERIODIC;
    forward.is_atomic = false;
    forward.default_steps = 4;
    forward.gait_period_ms = 1500;
    forward.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = 33;
    forward.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 33;
    forward.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = 25;
    forward.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]  = 25;
    forward.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)]  = 60;
    forward.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)]  = 60;
    forward.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
    forward.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
    forward.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = PI / 2;
    forward.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]  = PI / 2;
    forward.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]   = 0;
    forward.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)]  = 0;
    m_storage->save_action(forward);
    m_action_cache[forward.name] = forward;

    RegisteredAction backward = forward;
    strcpy(backward.name, "walk_backward");
    backward.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -33;
    backward.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -33;
    backward.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = 25;
    backward.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]  = 25;
    backward.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
    backward.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
    backward.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]   = PI / 2;
    backward.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)]  = PI / 2;
    m_storage->save_action(backward);
    m_action_cache[backward.name] = backward;

    RegisteredAction left = forward;
    strcpy(left.name, "turn_left");
    left.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -25;
    left.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 45;
    left.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = 25;
    left.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]  = -10;
    left.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
    left.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = -10;
    left.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]   = PI / 2;
    left.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = PI / 2;
    left.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]   = PI / 2;
    m_storage->save_action(left);
    m_action_cache[left.name] = left;

    RegisteredAction right = left;
    strcpy(right.name, "turn_right");
    right.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = 25;
    right.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -25;
    right.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = 20;
    right.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]  = 25;
    right.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -20;
    right.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -15;
    right.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]  = -10;
    right.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
    right.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 0;
    right.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = PI / 2;
    right.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = PI / 2;
    m_storage->save_action(right);
    m_action_cache[right.name] = right;

    RegisteredAction wiggle_ears = {};
    strcpy(wiggle_ears.name, "wiggle_ears");
    wiggle_ears.type = ActionType::GAIT_PERIODIC;
    wiggle_ears.is_atomic = false;
    wiggle_ears.default_steps = 2;
    wiggle_ears.gait_period_ms = 1500;
    wiggle_ears.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_LIFT)] = 15;
    wiggle_ears.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_LIFT)] = 15;
    wiggle_ears.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 10;
    wiggle_ears.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 10;
    m_storage->save_action(wiggle_ears);
    m_action_cache[wiggle_ears.name] = wiggle_ears;

    RegisteredAction wave_hand = {};
    strcpy(wave_hand.name, "wave_hand");
    wave_hand.type = ActionType::GAIT_PERIODIC;
    wave_hand.is_atomic = false;
    wave_hand.default_steps = 2;
    wave_hand.gait_period_ms = 1500;
    wave_hand.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)] = 30;
    wave_hand.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = 30;
    wave_hand.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = 20;
    wave_hand.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = 20;
    m_storage->save_action(wave_hand);
    m_action_cache[wave_hand.name] = wave_hand;

    RegisteredAction nod_head = {};
    strcpy(nod_head.name, "nod_head");
    nod_head.type = ActionType::GAIT_PERIODIC;
    nod_head.is_atomic = false;
    nod_head.default_steps = 2;
    nod_head.gait_period_ms = 1500;
    nod_head.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 20;
    m_storage->save_action(nod_head);
    m_action_cache[nod_head.name] = nod_head;

    RegisteredAction shake_head = nod_head;
    strcpy(shake_head.name, "shake_head");
    shake_head.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = 20;
    shake_head.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 0;
    m_storage->save_action(shake_head);
    m_action_cache[shake_head.name] = shake_head;

    RegisteredAction single_leg = {};
    strcpy(single_leg.name, "single_leg");
    single_leg.type = ActionType::GAIT_PERIODIC;
    single_leg.is_atomic = false;
    single_leg.default_steps = 2;
    single_leg.gait_period_ms = 1500;
    single_leg.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 1;
    single_leg.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 1; 
    single_leg.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 30;
    single_leg.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 30;
    m_storage->save_action(single_leg);
    m_action_cache[single_leg.name] = single_leg;

    RegisteredAction tracking_L = {};
    strcpy(tracking_L.name, "tracking_L");
    tracking_L.default_steps = 1;
    tracking_L.is_atomic = false;
    tracking_L.gait_period_ms = 1500;
    tracking_L.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = 40;
    tracking_L.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -40;
    tracking_L.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -30;
    tracking_L.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 35;
    tracking_L.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 20;
    tracking_L.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 20;
    tracking_L.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 12;
    tracking_L.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 12;
    tracking_L.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = PI / 2 + PI;
    tracking_L.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = PI;
    tracking_L.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = PI;
    tracking_L.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 2 * PI;
    m_storage->save_action(tracking_L); 
    m_action_cache[tracking_L.name] = tracking_L;

    auto tracking_R = tracking_L;
    strcpy(tracking_R.name, "tracking_R");
    tracking_R.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -40;
    tracking_R.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 40;
    tracking_R.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 35;
    tracking_R.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = -35;
    tracking_R.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 20;
    tracking_R.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 20;
    tracking_R.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 0;
    tracking_R.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 0;
    tracking_R.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 0;
    tracking_R.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = PI / 2;
    tracking_R.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = PI;
    tracking_R.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 0;
    m_storage->save_action(tracking_R);
    m_action_cache[tracking_R.name] = tracking_R;

 RegisteredAction dance = {};
    strcpy(dance.name, "dance");
    dance.type = ActionType::GAIT_PERIODIC;
    dance.is_atomic = false;            
    dance.default_steps = 4;            
    dance.gait_period_ms = 2000;        
    dance.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 30;
    dance.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 30;
    dance.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 40;
    dance.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = -40;
    dance.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -15;
    dance.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 15;
    dance.params.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = PI;
    m_storage->save_action(dance);
    m_action_cache[dance.name] = dance;
    ESP_LOGI(TAG, "Default actions created and cached.");
    
 RegisteredAction leg_swing = {};
    strcpy(leg_swing.name, "leg_swing");
    leg_swing.type = ActionType::GAIT_PERIODIC;
    leg_swing.is_atomic = false;
    leg_swing.default_steps = 3;
    leg_swing.gait_period_ms = 1200;
    leg_swing.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -50;
    leg_swing.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = -30;
    leg_swing.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 15;
    leg_swing.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 20;
    leg_swing.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 20;
    m_storage->save_action(leg_swing);
    m_action_cache[leg_swing.name] = leg_swing;
    ESP_LOGI(TAG, "Default actions created and cached.");

 RegisteredAction happy_wiggle = {};
    strcpy(happy_wiggle.name, "happy_wiggle");
    happy_wiggle.type = ActionType::GAIT_PERIODIC;
    happy_wiggle.is_atomic = false;
    happy_wiggle.default_steps = 4; 
    happy_wiggle.gait_period_ms = 800;  
    happy_wiggle.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10; 
    happy_wiggle.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10; 
    happy_wiggle.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 15;
    happy_wiggle.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 15;
    happy_wiggle.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = PI;
    happy_wiggle.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = 25;
    happy_wiggle.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 10;
    happy_wiggle.params.phase_diff[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = PI / 2;
    happy_wiggle.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 30;
    happy_wiggle.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 30;
    happy_wiggle.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = -45; 
    happy_wiggle.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = -45;
    happy_wiggle.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)] = 40;
    happy_wiggle.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = 40;
    happy_wiggle.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = PI;
    m_storage->save_action(happy_wiggle);
    m_action_cache[happy_wiggle.name] = happy_wiggle;
    ESP_LOGI(TAG, "Default actions created and cached.");

    RegisteredAction sad_stomp = {};
    strcpy(sad_stomp.name, "sad_stomp");
    sad_stomp.type = ActionType::GAIT_PERIODIC;
    sad_stomp.is_atomic = false;
    sad_stomp.default_steps = 4;      
    sad_stomp.gait_period_ms = 2200; 
    sad_stomp.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
    sad_stomp.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;  
    sad_stomp.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 15;
    sad_stomp.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 15;
    sad_stomp.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = PI;
    sad_stomp.params.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 30; 
    sad_stomp.params.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = 10;
    sad_stomp.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_EAR_LIFT)] = 20;
    sad_stomp.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_LIFT)] = 20;
    sad_stomp.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = -50; 
    sad_stomp.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = -50;
    sad_stomp.params.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)] = 30;  
    sad_stomp.params.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = -30; 
    sad_stomp.params.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = 5;
    sad_stomp.params.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = 5;
    sad_stomp.params.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = 0;
    m_storage->save_action(sad_stomp);
    m_action_cache[sad_stomp.name] = sad_stomp;
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
    action.gait_period_ms = gait_period_ms;
    ESP_LOGI(TAG, "Updated properties for action '%s': is_atomic=%s, steps=%d, period=%dms", 
             action_name.c_str(), is_atomic ? "true" : "false", (int)default_steps, (int)gait_period_ms);
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
    if (param_type == "amplitude") action.params.amplitude[servo_index] = value;
    else if (param_type == "offset") action.params.offset[servo_index] = value;
    else if (param_type == "phase_diff") action.params.phase_diff[servo_index] = value;
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
    std::string json_str;
    json_str.reserve(1024);
    char temp_buf[128];

    snprintf(temp_buf, sizeof(temp_buf), 
        "{\"name\":\"%s\",\"is_atomic\":%s,\"default_steps\":%d,\"gait_period_ms\":%d,\"params\":{", 
        action.name, action.is_atomic ? "true" : "false", (int)action.default_steps, (int)action.gait_period_ms);
    json_str += temp_buf;

    auto& params = action.params;
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
}

void ActionManager::print_action_details(const RegisteredAction &action) {
    ESP_LOGI(TAG, "[Action Details] Name: %s", action.name);
    ESP_LOGI(TAG, "  - Type: %s", action.type == ActionType::GAIT_PERIODIC ? "Gait Periodic" : "Unknown");
    ESP_LOGI(TAG, "  - Atomic: %s", action.is_atomic ? "Yes" : "No");
    ESP_LOGI(TAG, "  - Steps: %d", (int)action.default_steps);
    ESP_LOGI(TAG, "  - Period: %d ms", (int)action.gait_period_ms);
    
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

    print_float_array("Amplitude", action.params.amplitude);
    print_float_array("Offset   ", action.params.offset);
    print_float_array("Phase    ", action.params.phase_diff);
}  