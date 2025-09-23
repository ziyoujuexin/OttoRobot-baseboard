#include "AnimationPlayer.h"
#include "esp_log.h"

// Forward declaration of the command queue and structure from main.cpp
struct UiCommand {
    char animation_name[32];
};
extern QueueHandle_t ui_command_queue;

static const char* TAG = "AnimationPlayer";

// --- Queue Message Structure for internal player commands ---
struct PlayerCommand {
    char name[32];
};

// --- Constructor / Destructor ---
AnimationPlayer::AnimationPlayer(AnimationManager* anim_manager, DualScreenManager* display_manager)
    : m_anim_manager(anim_manager), m_display_manager(display_manager) { // display_manager is no longer used for UI updates
    if (!m_anim_manager) {
        ESP_LOGE(TAG, "AnimationPlayer received null AnimationManager!");
    }
}

AnimationPlayer::~AnimationPlayer() {
    if (m_task_handle) vTaskDelete(m_task_handle);
    if (m_player_queue) vQueueDelete(m_player_queue);
}

// --- Public API ---
void AnimationPlayer::start() {
    m_player_queue = xQueueCreate(5, sizeof(PlayerCommand));
    if (!m_player_queue) {
        ESP_LOGE(TAG, "Failed to create player queue!");
        return;
    }
    xTaskCreate(player_task_wrapper, "anim_player_task", 4096, this, 5, &m_task_handle);
    ESP_LOGI(TAG, "AnimationPlayer task started.");

    // Post a command to the UI task to play the initial default animation
    UiCommand initial_cmd;
    strncpy(initial_cmd.animation_name, "eyec", sizeof(initial_cmd.animation_name) - 1);
    initial_cmd.animation_name[sizeof(initial_cmd.animation_name) - 1] = '\0';
    xQueueSend(ui_command_queue, &initial_cmd, 0);
}

// This function is called by external tasks (like UartHandler) to request a one-shot animation
void AnimationPlayer::playOneShotAnimation(const std::string& animation_name) {
    PlayerCommand cmd;
    strncpy(cmd.name, animation_name.c_str(), sizeof(cmd.name) - 1);
    cmd.name[sizeof(cmd.name) - 1] = '\0';

    // Send the request to the player's own state machine task
    if (xQueueSend(m_player_queue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send one-shot command to player queue.");
    }
}

// --- Main Task Loop (State Machine) ---
void AnimationPlayer::player_task() {
    PlayerCommand received_cmd; // For commands from UartHandler etc.
    const TickType_t one_shot_duration = pdMS_TO_TICKS(5000); // 5 seconds for one-shot animations
    bool animation_needs_update = false;

    while (true) {
        // 1. Check for new one-shot animation requests
        if (xQueueReceive(m_player_queue, &received_cmd, 0) == pdPASS) {
            ESP_LOGI(TAG, "Player task received request: %s", received_cmd.name);
            m_current_state = PlayerState::PLAYING_ONESHOT;
            m_current_anim_name = received_cmd.name;
            m_one_shot_start_time = xTaskGetTickCount();
            animation_needs_update = true;
        } else {
            // 2. Process state machine if no new command
            if (m_current_state == PlayerState::PLAYING_ONESHOT) {
                if (xTaskGetTickCount() - m_one_shot_start_time >= one_shot_duration) {
                    ESP_LOGI(TAG, "One-shot '%s' finished, returning to default.", m_current_anim_name.c_str());
                    m_current_state = PlayerState::PLAYING_DEFAULT;
                    m_current_anim_name = "eyec";
                    animation_needs_update = true;
                }
            }
        }

        // 3. If the state changed, send an update command to the LVGL task
        if (animation_needs_update) {
            UiCommand ui_cmd;
            strncpy(ui_cmd.animation_name, m_current_anim_name.c_str(), sizeof(ui_cmd.animation_name) - 1);
            ui_cmd.animation_name[sizeof(ui_cmd.animation_name) - 1] = '\0';
            
            if (xQueueSend(ui_command_queue, &ui_cmd, 0) != pdPASS) {
                ESP_LOGE(TAG, "Failed to send command to UI queue from player task.");
            }
            animation_needs_update = false; // Reset flag
        }

        // 4. Wait for next cycle
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}