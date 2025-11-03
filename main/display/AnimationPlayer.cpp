#include "AnimationPlayer.h"
#include "esp_log.h"
#include "../UIManager.hpp" // Include UIManager to get the UiCommand definition

static const char* TAG = "AnimationPlayer";

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
    m_next_anim_mutex = xSemaphoreCreateMutex();
    if (!m_next_anim_mutex) {
        ESP_LOGE(TAG, "Failed to create animation player mutex!");
    }
}

AnimationPlayer::~AnimationPlayer() {
    if (m_task_handle) vTaskDelete(m_task_handle);
    if (m_next_anim_mutex) vSemaphoreDelete(m_next_anim_mutex);
}

// --- Public API ---
void AnimationPlayer::start() {
    xTaskCreatePinnedToCore(player_task_wrapper, "anim_player_task", 8192, this, 5, &m_task_handle, 0);
    ESP_LOGI(TAG, "AnimationPlayer task started.");
}

// This function is called by external tasks to schedule an animation for the next cycle.
// It overwrites any previously scheduled animation.
void AnimationPlayer::playOneShotAnimation(const std::string& animation_name) {
    if (xSemaphoreTake(m_next_anim_mutex, pdMS_TO_TICKS(100)) == pdPASS) {
        m_next_anim_name = animation_name;
        xSemaphoreGive(m_next_anim_mutex);
        ESP_LOGI(TAG, "Scheduled animation '%s' for next cycle.", animation_name.c_str());
    } else {
        ESP_LOGE(TAG, "Failed to schedule animation, could not lock mutex.");
    }
}

// --- Main Task Loop (Cyclical Playback) ---
void AnimationPlayer::player_task() {
    // The default animation to play when no other is queued
    const std::string default_animation = "中眨眼_2_69s";
    m_current_anim_name = default_animation;

    while (true) {
        // 1. Send command to UI task to play the current animation
        ESP_LOGD(TAG, "Playing animation cycle: %s", m_current_anim_name.c_str());
        UiCommand ui_cmd;
        strncpy(ui_cmd.animation_name, m_current_anim_name.c_str(), sizeof(ui_cmd.animation_name) - 1);
        ui_cmd.animation_name[sizeof(ui_cmd.animation_name) - 1] = '\0';
        
        if (xQueueSend(m_ui_command_queue, &ui_cmd, 0) != pdPASS) {
            ESP_LOGE(TAG, "Failed to send command to UI queue.");
            vTaskDelay(pdMS_TO_TICKS(1000)); // Avoid tight loop on queue error
            continue;
        }

        // 2. Calculate duration for the current animation
        TickType_t current_animation_duration = pdMS_TO_TICKS(5000); // Default duration if parsing fails
        float time_scale = 1.00f;
        std::string name_str(m_current_anim_name);

        const std::string extension = ".gif";
        if (name_str.length() >= extension.length() && name_str.substr(name_str.length() - extension.length()) == extension) {
            name_str.erase(name_str.length() - extension.length());
        }

        size_t last_underscore_pos = name_str.rfind('_');
        if (last_underscore_pos != std::string::npos && last_underscore_pos < name_str.length() - 1 && name_str[last_underscore_pos + 1] == 'x') {
            std::string scale_val_str = name_str.substr(last_underscore_pos + 2);
            char* end;
            const char* start = scale_val_str.c_str();
            float parsed_scale = strtof(start, &end);

            if (start != end && *end == '\0' && errno != ERANGE) {
                time_scale = parsed_scale;
                ESP_LOGD(TAG, "Animation '%s': Custom time scale 'x%s', factor %.2f.", m_current_anim_name.c_str(), scale_val_str.c_str(), time_scale);
            } else {
                ESP_LOGW(TAG, "Animation '%s': Found '_x' but failed to parse scale value '%s'.", m_current_anim_name.c_str(), scale_val_str.c_str());
            }
            name_str.erase(last_underscore_pos);
        }

        int underscore_count = 0;
        for (char c : name_str) {
            if (c == '_') underscore_count++;
        }

        bool duration_parsed = false;
        if (underscore_count > 0 && !name_str.empty() && name_str.back() == 's') {
            if (underscore_count == 1) { // e.g. name_3s
                size_t pos = name_str.find('_');
                std::string duration_val = name_str.substr(pos + 1, name_str.length() - pos - 2);
                char* end;
                const char* start = duration_val.c_str();
                long duration_sec_long = strtol(start, &end, 10);
                if (start != end && *end == '\0' && errno != ERANGE) {
                    current_animation_duration = pdMS_TO_TICKS(static_cast<uint32_t>(duration_sec_long * 1000.0f * time_scale));
                    duration_parsed = true;
                }
            } else if (underscore_count == 2) { // e.g. name_1_5s
                size_t first_pos = name_str.find('_');
                size_t second_pos = name_str.find('_', first_pos + 1);
                std::string int_part = name_str.substr(first_pos + 1, second_pos - first_pos - 1);
                std::string frac_part = name_str.substr(second_pos + 1, name_str.length() - second_pos - 2);
                std::string float_str = int_part + "." + frac_part;
                char* end;
                const char* start = float_str.c_str();
                float duration_sec = strtof(start, &end);
                if (start != end && *end == '\0' && errno != ERANGE) {
                    current_animation_duration = pdMS_TO_TICKS(static_cast<uint32_t>(duration_sec * 1000.0f * time_scale));
                    duration_parsed = true;
                }
            }
        }
        
        if (duration_parsed) {
             ESP_LOGD(TAG, "Animation '%s' cycle duration: %u ms (scaled).", m_current_anim_name.c_str(), pdTICKS_TO_MS(current_animation_duration));
        } else {
             ESP_LOGW(TAG, "Animation '%s': Could not parse duration, using default %u ms.", m_current_anim_name.c_str(), pdTICKS_TO_MS(current_animation_duration));
        }

        // 3. Wait for the animation cycle to finish
        vTaskDelay(current_animation_duration);

        // 4. Cycle finished, decide what to play next
        ESP_LOGD(TAG, "Animation cycle for '%s' finished. Checking for next animation.", m_current_anim_name.c_str());
        xSemaphoreTake(m_next_anim_mutex, portMAX_DELAY);
        
        if (!m_next_anim_name.empty()) {
            // An animation is waiting, play it next
            m_current_anim_name = m_next_anim_name;
            m_next_anim_name.clear(); // Clear the slot
        } else {
            // No animation waiting, revert to the default one
            m_current_anim_name = default_animation;
        }
        
        xSemaphoreGive(m_next_anim_mutex);
        // The loop will now restart with the new m_current_anim_name
    }
}