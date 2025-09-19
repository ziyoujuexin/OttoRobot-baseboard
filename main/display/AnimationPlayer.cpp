#include "AnimationPlayer.h"
#include "esp_log.h"

static const char* TAG = "AnimationPlayer";

// --- Queue Message Structure (Simplified) ---
struct PlayerCommand {
    char name[32];
};

// --- Constructor / Destructor ---
AnimationPlayer::AnimationPlayer(AnimationManager* anim_manager, DualScreenManager* display_manager, SemaphoreHandle_t lvgl_mutex)
    : m_anim_manager(anim_manager), m_display_manager(display_manager), m_lvgl_mutex(lvgl_mutex) {
    if (!m_anim_manager || !m_display_manager || !m_lvgl_mutex) {
        ESP_LOGE(TAG, "AnimationPlayer received null dependency!");
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
    ESP_LOGI(TAG, "AnimationPlayer task started (Simplified Mode).");
}

void AnimationPlayer::playOneShotAnimation(const std::string& animation_name) {
    PlayerCommand cmd;
    strncpy(cmd.name, animation_name.c_str(), sizeof(cmd.name) - 1);
    cmd.name[sizeof(cmd.name) - 1] = '\0';

    if (xQueueSend(m_player_queue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGE(TAG, "Failed to send one-shot command to queue.");
    }
}

// --- Main Task Loop (Simplified) ---
void AnimationPlayer::player_task() {
    PlayerCommand cmd;

    while (true) {
        // Wait indefinitely for an animation command
        if (xQueueReceive(m_player_queue, &cmd, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Received command to play: %s", cmd.name);
            
            std::string path = m_anim_manager->getAnimationPath(cmd.name);
            if (path.empty()) {
                ESP_LOGE(TAG, "Failed to get path for animation '%s'", cmd.name);
                continue; // Wait for next command
            }

            if (xSemaphoreTake(m_lvgl_mutex, portMAX_DELAY)) {
                ESP_LOGI(TAG, "Took LVGL mutex, updating animation source.");
                m_display_manager->UpdateAnimationSource(path);
                xSemaphoreGive(m_lvgl_mutex);
                ESP_LOGI(TAG, "Gave LVGL mutex.");
            }
            ESP_LOGI(TAG, "Finished playing %s, waiting for next command.", cmd.name);
        }
    }
}