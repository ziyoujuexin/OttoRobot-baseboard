#include "DualScreenManager.h"
#include "esp_log.h"

static const char* TAG = "DualScreenManager";

DualScreenManager::DualScreenManager() 
    : m_left_disp(get_left_screen_display()), 
      m_right_disp(get_right_screen_display()) {
    ESP_LOGI(TAG, "DualScreenManager initialized.");
    if (!m_left_disp || !m_right_disp) {
        ESP_LOGE(TAG, "Failed to get display handles!");
    }
}

DualScreenManager::~DualScreenManager() {
    ESP_LOGI(TAG, "DualScreenManager destroyed.");
}

void DualScreenManager::DisplayAnimation(ScreenId screen, const std::string& anim_path) {
    if (anim_path.empty()) {
        ESP_LOGE(TAG, "Received empty animation path.");
        return;
    }
    std::string srceen_str = (screen == SCREEN_LEFT) ? "LEFT" : (screen == SCREEN_RIGHT) ? "RIGHT" : "BOTH";
    ESP_LOGI(TAG, "Displaying animation on screen %s from path: %s", srceen_str.c_str(), anim_path.c_str());

    if (screen == SCREEN_LEFT || screen == SCREEN_BOTH) {
        if (m_left_disp) {
            create_anim_obj(m_left_disp, anim_path);
        }
    }
    if (screen == SCREEN_RIGHT || screen == SCREEN_BOTH) {
        if (m_right_disp) {
            create_anim_obj(m_right_disp, anim_path);
        }
    }
}

void DualScreenManager::create_anim_obj(lv_display_t* disp, const std::string& anim_path) {
    if (!disp) return;
    lv_obj_t* screen_obj = lv_display_get_screen_active(disp);
    if (!screen_obj) return;

    // Clear previous animation/object on this screen
    lv_obj_clean(screen_obj);

    // Create and display the new animation
    lv_obj_t* gif = lv_gif_create(screen_obj);
    lv_gif_set_src(gif, anim_path.c_str());
    lv_obj_align(gif, LV_ALIGN_CENTER, 0, 0);
}

void DualScreenManager::clear_disp(lv_display_t* disp) {
    if (!disp) return;
    lv_obj_t* screen_obj = lv_display_get_screen_active(disp);
    if (screen_obj) {
        lv_obj_clean(screen_obj);
    }
}

void DualScreenManager::ClearScreen(ScreenId screen) {
    ESP_LOGI(TAG, "Clearing screen %d", screen);
    if (screen == SCREEN_LEFT || screen == SCREEN_BOTH) {
        clear_disp(m_left_disp);
    }
    if (screen == SCREEN_RIGHT || screen == SCREEN_BOTH) {
        clear_disp(m_right_disp);
    }
}