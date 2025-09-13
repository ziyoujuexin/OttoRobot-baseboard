#include "DualScreenManager.h"
#include "esp_log.h"
#include "lvgl.h"

static const char* TAG = "DualScreenManager";

DualScreenManager::DualScreenManager() : m_left_anim_obj(nullptr), m_right_anim_obj(nullptr) {
    ESP_LOGI(TAG, "DualScreenManager initialized.");
}

DualScreenManager::~DualScreenManager() {
    if (m_left_anim_obj) { lv_obj_del(m_left_anim_obj); }
    if (m_right_anim_obj) { lv_obj_del(m_right_anim_obj); }
    ESP_LOGI(TAG, "DualScreenManager destroyed.");
}

void DualScreenManager::DisplayAnimation(ScreenId screen, const std::string& anim_path) {
    if (anim_path.empty()) {
        ESP_LOGE(TAG, "Received empty animation path.");
        return;
    }

    ESP_LOGI(TAG, "Displaying animation from path: %s", anim_path.c_str());

    if (screen == SCREEN_LEFT || screen == SCREEN_BOTH) {
        create_anim_obj(&m_left_anim_obj, anim_path);
    }
    if (screen == SCREEN_RIGHT || screen == SCREEN_BOTH) {
        create_anim_obj(&m_right_anim_obj, anim_path);
    }
}

void DualScreenManager::create_anim_obj(lv_obj_t** obj_ptr, const std::string& anim_path) {
    if (*obj_ptr != nullptr) {
        lv_obj_del(*obj_ptr);
        *obj_ptr = nullptr;
    }

    *obj_ptr = lv_gif_create(lv_scr_act());
    // LVGL can open the file path directly!
    lv_gif_set_src((*obj_ptr), anim_path.c_str());
    
    // You might want to add alignment and positioning here, for example:
    // lv_obj_align(*obj_ptr, LV_ALIGN_CENTER, 0, 0);
}

void DualScreenManager::ClearScreen(ScreenId screen) {
    if (screen == SCREEN_LEFT || screen == SCREEN_BOTH) {
        if (m_left_anim_obj) {
            lv_obj_del(m_left_anim_obj);
            m_left_anim_obj = nullptr;
        }
    }
    if (screen == SCREEN_RIGHT || screen == SCREEN_BOTH) {
        if (m_right_anim_obj) {
            lv_obj_del(m_right_anim_obj);
            m_right_anim_obj = nullptr;
        }
    }
}
