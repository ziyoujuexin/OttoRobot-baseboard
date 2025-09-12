#include "DualScreenManager.h"
#include "esp_log.h"

static const char* TAG = "DualScreenManager";

DualScreenManager::DualScreenManager() : m_left_anim_obj(nullptr), m_right_anim_obj(nullptr) {
    ESP_LOGI(TAG, "DualScreenManager initialized.");
}

DualScreenManager::~DualScreenManager() {
    // In a real implementation, ensure objects are deleted
    if (m_left_anim_obj) {
        // lv_obj_del(m_left_anim_obj);
    }
    if (m_right_anim_obj) {
        // lv_obj_del(m_right_anim_obj);
    }
    ESP_LOGI(TAG, "DualScreenManager destroyed.");
}

void DualScreenManager::DisplayAnimation(ScreenId screen, const AnimationData& anim_data) {
    if (!anim_data.IsValid()) {
        ESP_LOGE(TAG, "Received invalid animation data for animation '%s'", anim_data.name.c_str());
        return;
    }

    ESP_LOGI(TAG, "Displaying animation '%s'", anim_data.name.c_str());

    if (screen == SCREEN_LEFT || screen == SCREEN_BOTH) {
        create_anim_obj(&m_left_anim_obj, anim_data);
    }
    if (screen == SCREEN_RIGHT || screen == SCREEN_BOTH) {
        create_anim_obj(&m_right_anim_obj, anim_data);
    }
}

void DualScreenManager::create_anim_obj(lv_obj_t** obj_ptr, const AnimationData& anim_data) {
    // Delete the old object if it exists
    if (*obj_ptr != nullptr) {
        ESP_LOGI(TAG, "[DUMMY] Deleting old lv_obj (ptr: %p)", *obj_ptr);
        // In real implementation: lv_obj_del(*obj_ptr);
        *obj_ptr = nullptr;
    }

    ESP_LOGI(TAG, "[DUMMY] Creating new lv_gif for '%s'", anim_data.name.c_str());
    // In real implementation: *obj_ptr = lv_gif_create(lv_scr_act());
    *obj_ptr = (lv_obj_t*)0xBEEF0001; // Dummy pointer

    ESP_LOGI(TAG, "[DUMMY] Setting gif source from data buffer (size: %zu)", anim_data.data.size());
    // In real implementation: lv_gif_set_src(*obj_ptr, anim_data.data.data());
    // Note: LVGL might need a pointer to a struct that contains the data pointer and size,
    // or it might need the data to be in a specific format (e.g., a C-style array).
    // For GIF, lv_gif_set_src can take a path or a pointer to a variable.
    // If we pass a pointer to the data, we must ensure the data's lifetime
    // is longer than the object's. Since AnimationData is passed by const reference,
    // its lifetime is managed by the caller (AnimationManager), which is what we want.

    ESP_LOGI(TAG, "[DUMMY] New lv_obj created for '%s' (ptr: %p)", anim_data.name.c_str(), *obj_ptr);
}


void DualScreenManager::ClearScreen(ScreenId screen) {
    ESP_LOGI(TAG, "[DUMMY] ClearScreen called.");
    if (screen == SCREEN_LEFT || screen == SCREEN_BOTH) {
        if (m_left_anim_obj) {
            // lv_obj_del(m_left_anim_obj);
            m_left_anim_obj = nullptr;
        }
    }
    if (screen == SCREEN_RIGHT || screen == SCREEN_BOTH) {
        if (m_right_anim_obj) {
            // lv_obj_del(m_right_anim_obj);
            m_right_anim_obj = nullptr;
        }
    }
}