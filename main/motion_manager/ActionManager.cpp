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
    ESP_LOGW(TAG, "Temporarily erasing motion_db namespace...");
    nvs_handle_t nvs_handle;                                    
    if (nvs_open("motion_db", NVS_READWRITE, &nvs_handle) == ESP_OK) {                                                                                           
        nvs_erase_all(nvs_handle);                              
        nvs_commit(nvs_handle);                                 
        nvs_close(nvs_handle);                                  
        ESP_LOGW(TAG, "motion_db namespace erased.");           
    } else {                                                    
        ESP_LOGE(TAG, "Failed to open motion_db to erase.");
    }
    register_default_actions(false);
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

    RegisteredAction temp_action;
    if (!force && m_storage->load_action("walk_forward", temp_action)) {
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
        return;
    }

    ESP_LOGI(TAG, "Default actions not found in NVS. Creating them...");

    RegisteredAction forward = {};
    strcpy(forward.name, "walk_forward");
    forward.type = ActionType::GAIT_PERIODIC;
    forward.is_atomic = false;
    forward.default_steps = 4;
    forward.gait_period_ms = 1500;
    motion_params_t forward_wave = {};
    forward_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = 33;
    forward_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 33;
    forward_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = 25;
    forward_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]  = 25;
    forward_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)]  = 60;
    forward_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)]  = 60;
    forward_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
    forward_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
    forward_wave.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = PI / 2;
    forward_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]  = PI / 2;
    forward_wave.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]   = 0;
    forward_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)]  = 0;
    forward.harmonic_terms.push_back(forward_wave);
    forward.easing_type = EasingType::LINEAR;
    m_storage->save_action(forward);
    m_action_cache[forward.name] = forward;

    RegisteredAction backward = {};
    strcpy(backward.name, "walk_backward");
    backward.type = ActionType::GAIT_PERIODIC;
    backward.is_atomic = false;
    backward.default_steps = 4;
    backward.gait_period_ms = 1500;
    motion_params_t backward_wave = {};
    backward_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -33;
    backward_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -33;
    backward_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = 25;
    backward_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]  = 25;
    backward_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
    backward_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
    backward_wave.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]   = PI / 2;
    backward_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)]  = PI / 2;
    backward.harmonic_terms.push_back(backward_wave);
    backward.easing_type = EasingType::LINEAR;
    m_storage->save_action(backward);
    m_action_cache[backward.name] = backward;

    RegisteredAction left = {};
    strcpy(left.name, "turn_left");
    left.type = ActionType::GAIT_PERIODIC;
    left.is_atomic = false;
    left.default_steps = 4;
    left.gait_period_ms = 1500;
    motion_params_t left_wave = {};
    left_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -25;
    left_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 45;
    left_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = 25;
    left_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]  = -10;
    left_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
    left_wave.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = -10;
    left_wave.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]   = PI / 2;
    left_wave.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = PI / 2;
    left_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]   = PI / 2;
    left.harmonic_terms.push_back(left_wave);
    left.easing_type = EasingType::LINEAR;
    m_storage->save_action(left);
    m_action_cache[left.name] = left;

    RegisteredAction right = {};
    strcpy(right.name, "turn_right");
    right.type = ActionType::GAIT_PERIODIC;
    right.is_atomic = false;
    right.default_steps = 4;
    right.gait_period_ms = 1500;
    motion_params_t right_wave = {};
    right_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = 25;
    right_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -25;
    right_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = 20;
    right_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)]  = 25;
    right_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -20;
    right_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -15;
    right_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]  = -10;
    right_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
    right_wave.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 0;
    right_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = PI / 2;
    right_wave.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)]   = PI / 2;
    right.harmonic_terms.push_back(right_wave);
    right.easing_type = EasingType::LINEAR;
    m_storage->save_action(right);
    m_action_cache[right.name] = right;

    RegisteredAction wiggle_ears = {};
    strcpy(wiggle_ears.name, "wiggle_ears");
    wiggle_ears.type = ActionType::GAIT_PERIODIC;
    wiggle_ears.is_atomic = false;
    wiggle_ears.default_steps = 2;
    wiggle_ears.gait_period_ms = 1500;
    motion_params_t wiggle_ears_wave = {};
    wiggle_ears_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_LIFT)] = 15;
    wiggle_ears_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_LIFT)] = 15;
    wiggle_ears_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 10;
    wiggle_ears_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 10;
    wiggle_ears.harmonic_terms.push_back(wiggle_ears_wave);
    wiggle_ears.easing_type = EasingType::LINEAR;
    m_storage->save_action(wiggle_ears);
    m_action_cache[wiggle_ears.name] = wiggle_ears;

    RegisteredAction wave_hand = {};
    strcpy(wave_hand.name, "wave_hand");
    wave_hand.type = ActionType::GAIT_PERIODIC;
    wave_hand.is_atomic = false;
    wave_hand.default_steps = 2;
    wave_hand.gait_period_ms = 1500;
    motion_params_t wave_hand_wave = {};
    wave_hand_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)] = 30;
    wave_hand_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = 30;
    wave_hand_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = 20;
    wave_hand_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = 20;
    wave_hand.harmonic_terms.push_back(wave_hand_wave);
    wave_hand.easing_type = EasingType::LINEAR;
    m_storage->save_action(wave_hand);
    m_action_cache[wave_hand.name] = wave_hand;

    RegisteredAction nod_head = {};
    strcpy(nod_head.name, "nod_head");
    nod_head.type = ActionType::GAIT_PERIODIC;
    nod_head.is_atomic = false;
    nod_head.default_steps = 2;
    nod_head.gait_period_ms = 1500;
    motion_params_t nod_head_wave = {};
    nod_head_wave.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 20;
    nod_head.harmonic_terms.push_back(nod_head_wave);
    nod_head.easing_type = EasingType::LINEAR;
    m_storage->save_action(nod_head);
    m_action_cache[nod_head.name] = nod_head;

    RegisteredAction shake_head = {};
    strcpy(shake_head.name, "shake_head");
    shake_head.type = ActionType::GAIT_PERIODIC;
    shake_head.is_atomic = false;
    shake_head.default_steps = 2;
    shake_head.gait_period_ms = 1500;
    motion_params_t shake_head_wave = {};
    shake_head_wave.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = 20;
    shake_head.harmonic_terms.push_back(shake_head_wave);
    shake_head.easing_type = EasingType::LINEAR;
    m_storage->save_action(shake_head);
    m_action_cache[shake_head.name] = shake_head;

    RegisteredAction single_leg = {};
    strcpy(single_leg.name, "single_leg");
    single_leg.type = ActionType::GAIT_PERIODIC;
    single_leg.is_atomic = false;
    single_leg.default_steps = 2;
    single_leg.gait_period_ms = 1500;
    motion_params_t single_leg_wave = {};
    single_leg_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 1;
    single_leg_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 1; 
    single_leg_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 30;
    single_leg_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 30;
    single_leg.harmonic_terms.push_back(single_leg_wave);
    single_leg.easing_type = EasingType::LINEAR;
    m_storage->save_action(single_leg);
    m_action_cache[single_leg.name] = single_leg;

    RegisteredAction silly = {};
    strcpy(silly.name, "silly");
    silly.type = ActionType::GAIT_PERIODIC;
    silly.is_atomic = false;       
    silly.default_steps = 4;            
    silly.gait_period_ms = 2000;
    motion_params_t silly_wave = {};
    silly_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 30;
    silly_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 30;
    silly_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 40;
    silly_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = -40;
    silly_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -15;
    silly_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 15;
    silly_wave.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = PI;
    silly.harmonic_terms.push_back(silly_wave);
    silly.easing_type = EasingType::LINEAR;
    m_storage->save_action(silly);
    m_action_cache[silly.name] = silly;
    
   RegisteredAction funny = {};
    strcpy(funny.name, "funny");
    funny.type = ActionType::GAIT_PERIODIC;
    funny.is_atomic = false;
    funny.default_steps = 3;
    funny.gait_period_ms = 1200;
    motion_params_t funny_wave = {};
    funny_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -50;
    funny_wave.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = -30;
    funny_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 15;
    funny_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 20;
    funny_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 20;
    funny.harmonic_terms.push_back(funny_wave);
    funny.easing_type = EasingType::LINEAR;
    m_storage->save_action(funny);
    m_action_cache[funny.name] = funny;

    RegisteredAction happy = {};
    strcpy(happy.name, "happy");
    happy.type = ActionType::GAIT_PERIODIC;
    happy.is_atomic = false;
    happy.default_steps = 8;
    happy.gait_period_ms = 800;
    motion_params_t happy_wave = {};
    happy_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
    happy_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
    happy_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 15;
    happy_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 15;
    happy_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = PI;
    happy_wave.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = 25;
    happy_wave.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 10;
    happy_wave.phase_diff[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = PI / 2;
    happy_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 30;
    happy_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 30;
    happy_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = -45;
    happy_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = -45;
    happy_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)] = 40;
    happy_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = 40;
    happy_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = PI;
    happy.harmonic_terms.push_back(happy_wave);
    happy.easing_type = EasingType::LINEAR;
    m_storage->save_action(happy);
    m_action_cache[happy.name] = happy;

    RegisteredAction sad = {};
    strcpy(sad.name, "sad");
    sad.type = ActionType::GAIT_PERIODIC;
    sad.is_atomic = false;
    sad.default_steps = 4;
    sad.gait_period_ms = 2200;
    motion_params_t sad_wave = {};
    sad_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
    sad_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
    sad_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 15;
    sad_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 15;
    sad_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = PI;
    sad_wave.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 30;
    sad_wave.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = 10;
    sad_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_EAR_LIFT)] = 20;
    sad_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_LIFT)] = 20;
    sad_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = -50;
    sad_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = -50;
    sad_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)] = 30;
    sad_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = -30;
    sad_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = 5;
    sad_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = 5;
    sad_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = 0;
    sad.harmonic_terms.push_back(sad_wave);
    sad.easing_type = EasingType::LINEAR;
    m_storage->save_action(sad);
    m_action_cache[sad.name] = sad;

    RegisteredAction laughing = {};
    strcpy(laughing.name, "laughing");
    laughing.type = ActionType::GAIT_PERIODIC;
    laughing.is_atomic = false;
    laughing.default_steps = 6;
    laughing.gait_period_ms = 1000;
    motion_params_t laughing_wave = {};
    laughing_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
    laughing_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
    laughing_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 15;
    laughing_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 15;
    laughing_wave.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = -25;
    laughing_wave.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 10;
    laughing_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 40;
    laughing_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = -40;
    laughing.harmonic_terms.push_back(laughing_wave);
    laughing.easing_type = EasingType::LINEAR;
    m_storage->save_action(laughing);
    m_action_cache[laughing.name] = laughing;

    RegisteredAction angry = {};
    strcpy(angry.name, "angry");
    angry.type = ActionType::GAIT_PERIODIC;
    angry.is_atomic = true;
    angry.default_steps = 4;
    angry.gait_period_ms = 900;
    motion_params_t angry_wave = {};
    angry_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
    angry_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
    angry_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 20;
    angry_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 20;
    angry_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = PI;
    angry_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 20;
    angry_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 20;
    angry_wave.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 20;
    angry_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = -30;
    angry_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 30;
    angry.harmonic_terms.push_back(angry_wave);
    angry.easing_type = EasingType::LINEAR;
    m_storage->save_action(angry);
    m_action_cache[angry.name] = angry;

    RegisteredAction crying = {};
    strcpy(crying.name, "crying");
    crying.type = ActionType::GAIT_PERIODIC;
    crying.is_atomic = false;
    crying.default_steps = 5;
    crying.gait_period_ms = 2500;
    motion_params_t crying_wave = {};
    crying_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
    crying_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
    crying_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 8;
    crying_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 8;
    crying_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = PI;
    crying_wave.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 40;
    crying_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = -60;
    crying_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = -60;
    crying_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)] = 20;
    crying_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = 20;
    crying_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = PI;
    crying.harmonic_terms.push_back(crying_wave);
    crying.easing_type = EasingType::LINEAR;
    m_storage->save_action(crying);
    m_action_cache[crying.name] = crying;

    RegisteredAction surprised = {};
    strcpy(surprised.name, "surprised");
    surprised.type = ActionType::GAIT_PERIODIC;
    surprised.is_atomic = false;
    surprised.default_steps = 1;      
    surprised.gait_period_ms = 1200; 
    motion_params_t surprised_wave = {};
    surprised_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
    surprised_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
    surprised_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = -15;
    surprised_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -15;
    surprised_wave.offset[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = -20;
    surprised_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_EAR_LIFT)] = -30;
    surprised_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_LIFT)] = -30;
    surprised_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = -30;
    surprised_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_LIFT)] = -30;
    surprised_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)] = -20;
    surprised_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ARM_SWING)] = 20;
    surprised.harmonic_terms.push_back(surprised_wave);
    surprised.easing_type = EasingType::LINEAR;
    m_storage->save_action(surprised);
    m_action_cache[surprised.name] = surprised;


    RegisteredAction thinking = {};
    strcpy(thinking.name, "thinking");
    thinking.type = ActionType::GAIT_PERIODIC;
    thinking.is_atomic = false;
    thinking.default_steps = 4;
    thinking.gait_period_ms = 3000; 
    motion_params_t thinking_wave = {};
    thinking_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -10;
    thinking_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 10;
    thinking_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 15;
    thinking_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 15;
    thinking_wave.offset[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = 25;
    thinking_wave.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_PAN)] = 5;
    thinking_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_EAR_SWING)] = 30;
    thinking_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_EAR_SWING)] = 5;
    thinking_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 5;
    thinking_wave.amplitude[static_cast<uint8_t>(ServoChannel::HEAD_TILT)] = 5;
    thinking_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_LIFT)] = -70; 
    thinking_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ARM_SWING)] = 20;  
    thinking.harmonic_terms.push_back(thinking_wave);
    thinking.easing_type = EasingType::LINEAR;
    m_storage->save_action(thinking);
    m_action_cache[thinking.name] = thinking;

    RegisteredAction tracking_L = {};
    strcpy(tracking_L.name, "tracking_L");
    tracking_L.default_steps = 1;
    tracking_L.is_atomic = false;
    tracking_L.gait_period_ms = 1500;
    motion_params_t tracking_L_wave = {};
    tracking_L_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = 40;
    tracking_L_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = -40;
    tracking_L_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = -30;
    tracking_L_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 35;
    tracking_L_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 20;
    tracking_L_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 20;
    tracking_L_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 12;
    tracking_L_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 12;
    tracking_L_wave.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = PI / 2 + PI;
    tracking_L_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = PI;
    tracking_L_wave.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = PI;
    tracking_L_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 2 * PI;
    tracking_L.harmonic_terms.push_back(tracking_L_wave);
    tracking_L.easing_type = EasingType::LINEAR;
    m_storage->save_action(tracking_L);
    m_action_cache[tracking_L.name] = tracking_L;

    auto tracking_R = tracking_L;
    strcpy(tracking_R.name, "tracking_R");
    motion_params_t tracking_R_wave = {};
    tracking_R_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)]  = -40;
    tracking_R_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 40;
    tracking_R_wave.amplitude[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 35;
    tracking_R_wave.amplitude[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = -35;
    tracking_R_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 20;
    tracking_R_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = 20;
    tracking_R_wave.offset[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = 0;
    tracking_R_wave.offset[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 0;
    tracking_R_wave.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_LEG_ROTATE)] = 0;
    tracking_R_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_LEG_ROTATE)] = PI / 2;
    tracking_R_wave.phase_diff[static_cast<uint8_t>(ServoChannel::LEFT_ANKLE_LIFT)] = PI;
    tracking_R_wave.phase_diff[static_cast<uint8_t>(ServoChannel::RIGHT_ANKLE_LIFT)] = 0;
    tracking_R.harmonic_terms.clear();
    tracking_R.harmonic_terms.push_back(tracking_R_wave);
    tracking_R.easing_type = EasingType::LINEAR;
    m_storage->save_action(tracking_R);
    m_action_cache[tracking_R.name] = tracking_R;
    
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

bool ActionManager::update_action_properties(const std::string& action_name, bool is_atomic, uint32_t default_steps, uint32_t gait_period_ms, EasingType new_easing_type) {
    auto it = m_action_cache.find(action_name);
    if (it == m_action_cache.end()) {
        ESP_LOGE(TAG, "Action '%s' not found in cache for property update.", action_name.c_str());
        return false;
    }
    RegisteredAction& action = it->second;
    action.is_atomic = is_atomic;
    action.default_steps = default_steps;
    action.gait_period_ms = gait_period_ms;
    action.easing_type = new_easing_type;
    ESP_LOGI(TAG, "Updated properties for action '%s': is_atomic=%s, steps=%d, period=%dms, easing=%d", 
             action_name.c_str(), is_atomic ? "true" : "false", (int)default_steps, (int)gait_period_ms, static_cast<int>(new_easing_type));
    print_action_details(action);
    return true;
}

bool ActionManager::tune_gait_parameter(const std::string& action_name, int servo_index, const std::string& param_type, float value, int harmonic_index) {
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
    if (harmonic_index >= 0 && harmonic_index < action.harmonic_terms.size()) {
        if (param_type == "amplitude") action.harmonic_terms[harmonic_index].amplitude[servo_index] = value;
        else if (param_type == "offset") action.harmonic_terms[harmonic_index].offset[servo_index] = value;
        else if (param_type == "phase_diff") action.harmonic_terms[harmonic_index].phase_diff[servo_index] = value;
        else {
            ESP_LOGE(TAG, "Unknown parameter type: %s", param_type.c_str());
            return false;
        }
    } else {
        ESP_LOGE(TAG, "Invalid harmonic_index: %d", harmonic_index);
        return false;
    }
    ESP_LOGI(TAG, "Tuned %s for %s, servo %d, harmonic %d: set to %.2f", param_type.c_str(), action_name.c_str(), servo_index, harmonic_index, value);
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
    json_str.reserve(2048);
    char temp_buf[256];

    snprintf(temp_buf, sizeof(temp_buf), 
        "{\"name\":\"%s\",\"is_atomic\":%s,\"default_steps\":%d,\"gait_period_ms\":%d,\"easing_type\":%d,\"harmonic_terms\":[", 
        action.name, action.is_atomic ? "true" : "false", (int)action.default_steps, (int)action.gait_period_ms, static_cast<int>(action.easing_type));
    json_str += temp_buf;

    for (size_t j = 0; j < action.harmonic_terms.size(); ++j) {
        const auto& term = action.harmonic_terms[j];
        json_str += "{";

        json_str += "\"amplitude\":[";
        for(int i=0; i<GAIT_JOINT_COUNT; ++i) {
            snprintf(temp_buf, sizeof(temp_buf), "%.2f", term.amplitude[i]);
            json_str += temp_buf;
            if (i < GAIT_JOINT_COUNT - 1) json_str += ",";
        }
        json_str += "],";

        json_str += "\"offset\":[";
        for(int i=0; i<GAIT_JOINT_COUNT; ++i) {
            snprintf(temp_buf, sizeof(temp_buf), "%.2f", term.offset[i]);
            json_str += temp_buf;
            if (i < GAIT_JOINT_COUNT - 1) json_str += ",";
        }
        json_str += "],";

        json_str += "\"phase_diff\":[";
        for(int i=0; i<GAIT_JOINT_COUNT; ++i) {
            snprintf(temp_buf, sizeof(temp_buf), "%.2f", term.phase_diff[i]);
            json_str += temp_buf;
            if (i < GAIT_JOINT_COUNT - 1) json_str += ",";
        }
        json_str += "]}";
        if (j < action.harmonic_terms.size() - 1) {
            json_str += ",";
        }
    }

    json_str += "]}";
    return json_str;
}

void ActionManager::print_action_details(const RegisteredAction &action) {
    ESP_LOGI(TAG, "[Action Details] Name: %s", action.name);
    ESP_LOGI(TAG, "  - Type: %s", action.type == ActionType::GAIT_PERIODIC ? "Gait Periodic" : "Unknown");
    ESP_LOGI(TAG, "  - Atomic: %s", action.is_atomic ? "Yes" : "No");
    ESP_LOGI(TAG, "  - Steps: %d", (int)action.default_steps);
    ESP_LOGI(TAG, "  - Period: %d ms", (int)action.gait_period_ms);
    ESP_LOGI(TAG, "  - Easing: %d", static_cast<int>(action.easing_type));
    
    auto print_float_array = [](const char* prefix, const float* arr) {
        char buffer[256];
        int offset = snprintf(buffer, sizeof(buffer), "    - %s: [", prefix);
        for(int i=0; i<GAIT_JOINT_COUNT; ++i) {
            offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%.2f, ", arr[i]);
        }
        if (offset > 2) { 
            buffer[offset-2] = ']';
            buffer[offset-1] = '\0';
        }
        ESP_LOGI(TAG, "%s", buffer);
    };

    int i = 0;
    for (const auto& term : action.harmonic_terms) {
        ESP_LOGI(TAG, "  - Harmonic Term %d:", i++);
        print_float_array("Amplitude", term.amplitude);
        print_float_array("Offset   ", term.offset);
        print_float_array("Phase    ", term.phase_diff);
    }
}  