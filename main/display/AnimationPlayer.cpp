#include "AnimationPlayer.h"
#include "esp_log.h"
#include "../UIManager.hpp" // Include UIManager to get the UiCommand definition

static const char* TAG = "AnimationPlayer";

// --- Queue Message Structure for internal player commands ---
struct PlayerCommand {
    char name[32];
};

// --- Constructor / Destructor ---
AnimationPlayer::AnimationPlayer(AnimationManager* anim_manager, DualScreenManager* display_manager, QueueHandle_t ui_command_queue)
    : m_anim_manager(anim_manager), 
      m_display_manager(display_manager), 
      m_ui_command_queue(ui_command_queue) { 
    if (!m_anim_manager) {
        ESP_LOGE(TAG, "AnimationPlayer received null AnimationManager!");
    }
    if (!m_ui_command_queue) {
        ESP_LOGE(TAG, "AnimationPlayer received null ui_command_queue!");
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
    xTaskCreate(player_task_wrapper, "anim_player_task", 8192, this, 5, &m_task_handle);
    ESP_LOGI(TAG, "AnimationPlayer task started.");

    // Post a command to the UI task to play the initial default animation
    UiCommand initial_cmd;
    strncpy(initial_cmd.animation_name, "中眨眼_2_69s", sizeof(initial_cmd.animation_name) - 1);
    initial_cmd.animation_name[sizeof(initial_cmd.animation_name) - 1] = '\0';
    xQueueSend(m_ui_command_queue, &initial_cmd, 0);
}

// This function is called by external tasks (like UartHandler) to request a one-shot animation
void AnimationPlayer::playOneShotAnimation(const std::string& animation_name) {
    PlayerCommand cmd;
    strncpy(cmd.name, animation_name.c_str(), sizeof(cmd.name) - 1);
    cmd.name[sizeof(cmd.name) - 1] = '\0';

    // Send the request to the player's own state machine task
    if (!m_player_queue || xQueueSend(m_player_queue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send one-shot command to player queue.");
    }
}

// --- Main Task Loop (State Machine) ---
void AnimationPlayer::player_task() {
    PlayerCommand received_cmd; // For commands from UartHandler etc.
    TickType_t current_one_shot_duration = pdMS_TO_TICKS(5000); // Default duration
    bool animation_needs_update = false;

    while (true) {
        // 1. Check for new one-shot animation requests
        if (xQueueReceive(m_player_queue, &received_cmd, 0) == pdPASS) {
            ESP_LOGI(TAG, "Player task received request: %s", received_cmd.name);
            m_current_state = PlayerState::PLAYING_ONESHOT;
            m_current_anim_name = received_cmd.name;
            m_one_shot_start_time = xTaskGetTickCount();
            animation_needs_update = true;

            // --- Animation Duration Parsing Logic ---
            current_one_shot_duration = pdMS_TO_TICKS(5000); // Default duration

            std::string name_str(m_current_anim_name);

            // 1. Strip .gif extension if it exists
            const std::string extension = ".gif";
            if (name_str.length() >= extension.length() && name_str.substr(name_str.length() - extension.length()) == extension) {
                name_str.erase(name_str.length() - extension.length());
            }

            // 2. Count underscores
            int underscore_count = 0;
            for (char c : name_str) {
                if (c == '_') {
                    underscore_count++;
                }
            }

            // 3. Apply rules based on underscore count
            if (underscore_count == 0) {
                ESP_LOGI(TAG, "Animation '%s': 0 underscores, using default 5s duration.", m_current_anim_name.c_str());
            } else if (name_str.back() != 's') {
                ESP_LOGW(TAG, "Animation '%s': Invalid format (must end with 's'), using default 5s.", m_current_anim_name.c_str());
            } else if (underscore_count == 1) {
                // Case: name_3s
                size_t pos = name_str.find('_');
                std::string duration_val = name_str.substr(pos + 1, name_str.length() - pos - 2);
                
                char* end;
                const char* start = duration_val.c_str();
                long duration_sec_long = strtol(start, &end, 10);

                if (start == end || *end != '\0' || errno == ERANGE) {
                    ESP_LOGW(TAG, "Animation '%s': Invalid integer format for duration '%s'. Using default 5s.", m_current_anim_name.c_str(), duration_val.c_str());
                } else {
                    current_one_shot_duration = pdMS_TO_TICKS(static_cast<int>(duration_sec_long) * 1000.0f * m_time_scale_percent);
                    ESP_LOGI(TAG, "Animation '%s': Parsed integer duration: %d seconds.", m_current_anim_name.c_str(), static_cast<int>(duration_sec_long));
                }
            } else if (underscore_count == 2) {
                // Case: name_1_5s
                size_t first_pos = name_str.find('_');
                size_t second_pos = name_str.find('_', first_pos + 1);
                std::string int_part = name_str.substr(first_pos + 1, second_pos - first_pos - 1);
                std::string frac_part = name_str.substr(second_pos + 1, name_str.length() - second_pos - 2);
                std::string float_str = int_part + "." + frac_part;

                char* end;
                const char* start = float_str.c_str();
                float duration_sec = strtof(start, &end);

                if (start == end || *end != '\0' || errno == ERANGE) {
                    ESP_LOGW(TAG, "Animation '%s': Invalid float format for duration '%s'. Using default 5s.", m_current_anim_name.c_str(), float_str.c_str());
                } else {
                    current_one_shot_duration = pdMS_TO_TICKS(static_cast<uint32_t>(duration_sec * 1000.0f * m_time_scale_percent));
                    ESP_LOGI(TAG, "Animation '%s': Parsed float duration: %.2f seconds.", m_current_anim_name.c_str(), duration_sec);
                }
            } else {
                ESP_LOGW(TAG, "Animation '%s': Invalid format (>2 underscores), using default 5s.", m_current_anim_name.c_str());
            }

        } else {
            // 2. Process state machine if no new command
            if (m_current_state == PlayerState::PLAYING_ONESHOT) {
                if (xTaskGetTickCount() - m_one_shot_start_time >= current_one_shot_duration) {
                    ESP_LOGI(TAG, "One-shot '%s' finished, returning to default.", m_current_anim_name.c_str());
                    m_current_state = PlayerState::PLAYING_DEFAULT;
                    m_current_anim_name = "中眨眼_2_69s";
                    animation_needs_update = true;
                }
            }
        }

        // 3. If the state changed, send an update command to the LVGL task
        if (animation_needs_update) {
            UiCommand ui_cmd;
            strncpy(ui_cmd.animation_name, m_current_anim_name.c_str(), sizeof(ui_cmd.animation_name) - 1);
            ui_cmd.animation_name[sizeof(ui_cmd.animation_name) - 1] = '\0';
            
            if (xQueueSend(m_ui_command_queue, &ui_cmd, 0) != pdPASS) {
                ESP_LOGE(TAG, "Failed to send command to UI queue from player task.");
            }
            animation_needs_update = false; // Reset flag
        }

        // 4. Wait for next cycle
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}