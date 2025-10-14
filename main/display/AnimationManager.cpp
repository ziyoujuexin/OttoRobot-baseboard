#include "AnimationManager.h"
#include "esp_log.h"

static const char* TAG = "AnimationManager";

AnimationManager::AnimationManager(std::unique_ptr<AnimationProvider> provider)
    : m_provider(std::move(provider)) {
    if (!m_provider) {
        ESP_LOGE(TAG, "AnimationProvider cannot be null!");
    }
}

AnimationPair AnimationManager::getAnimationData(const std::string& animation_name) {
    if (!m_provider) {
        ESP_LOGE(TAG, "Manager is not properly initialized.");
        return {}; // Return invalid AnimationPair
    }
    return m_provider->getAnimationData(animation_name);
}

void AnimationManager::releaseAnimationData(AnimationData& anim_data) {
    if (!m_provider) {
        ESP_LOGE(TAG, "Manager is not properly initialized.");
        return;
    }
    m_provider->releaseAnimationData(anim_data);
}

void AnimationManager::releaseAnimationPair(AnimationPair& anim_pair) {
    if (!m_provider) {
        ESP_LOGE(TAG, "Manager is not properly initialized.");
        return;
    }
    m_provider->releaseAnimationPair(anim_pair);
}
