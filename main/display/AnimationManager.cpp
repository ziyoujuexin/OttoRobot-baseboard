#include "AnimationManager.h"
#include "esp_log.h"

static const char* TAG = "AnimationManager";

AnimationManager::AnimationManager(std::unique_ptr<AnimationProvider> provider)
    : m_provider(std::move(provider)) {
    if (!m_provider) {
        ESP_LOGE(TAG, "AnimationProvider cannot be null!");
    }
}

std::string AnimationManager::getAnimationPath(const std::string& animation_name) {
    if (!m_provider) {
        ESP_LOGE(TAG, "Manager is not properly initialized.");
        return "";
    }
    return m_provider->getAnimationPath(animation_name);
}
