#include "AnimationManager.h"
#include "esp_log.h"

static const char* TAG = "AnimationManager";

AnimationManager::AnimationManager(std::unique_ptr<AnimationProvider> provider, DualScreenManager* display_manager)
    : m_provider(std::move(provider)), m_display_manager(display_manager) {
    if (!m_provider) {
        ESP_LOGE(TAG, "AnimationProvider cannot be null!");
    }
    if (!m_display_manager) {
        ESP_LOGE(TAG, "DualScreenManager cannot be null!");
    }
}

bool AnimationManager::PlayAnimation(const std::string& animation_name, ScreenId screen) {
    if (!m_provider || !m_display_manager) {
        ESP_LOGE(TAG, "Manager is not properly initialized.");
        return false;
    }

    ESP_LOGI(TAG, "Requesting to play animation '%s'...", animation_name.c_str());

    // 1. Use the provider to get the animation path
    m_current_lvgl_path = m_provider->getAnimationPath(animation_name);

    if (m_current_lvgl_path.empty()) {
        ESP_LOGE(TAG, "Failed to get path for animation '%s'.", animation_name.c_str());
        return false;
    }

    // 2. Pass the path to the display manager to be shown
    m_display_manager->DisplayAnimation(screen, m_current_lvgl_path);

    return true;
}
