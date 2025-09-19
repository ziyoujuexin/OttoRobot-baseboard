#include "DualScreenManager.h"
#include "esp_log.h"
#include "freertos/task.h"

static const char* TAG = "DualScreenManager";

DualScreenManager::DualScreenManager() 
    : m_left_disp(get_left_screen_display()), 
      m_right_disp(get_right_screen_display()),
      m_left_gif_obj(nullptr),
      m_right_gif_obj(nullptr) {
    ESP_LOGI(TAG, "DualScreenManager initialized.");
    if (!m_left_disp || !m_right_disp) {
        ESP_LOGE(TAG, "Failed to get display handles!");
        return;
    }

    // Create the GIF objects once at the beginning
    lv_obj_t* left_screen_obj = lv_display_get_screen_active(m_left_disp);
    if (left_screen_obj) {
        m_left_gif_obj = lv_gif_create(left_screen_obj);
        lv_obj_align(m_left_gif_obj, LV_ALIGN_CENTER, 0, 0);
        ESP_LOGI(TAG, "Left GIF object created.");
    }

    lv_obj_t* right_screen_obj = lv_display_get_screen_active(m_right_disp);
    if (right_screen_obj) {
        m_right_gif_obj = lv_gif_create(right_screen_obj);
        lv_obj_align(m_right_gif_obj, LV_ALIGN_CENTER, 0, 0);
        ESP_LOGI(TAG, "Right GIF object created.");
    }
}

DualScreenManager::~DualScreenManager() {
    ESP_LOGI(TAG, "DualScreenManager destroyed.");
    // lv_obj_del will be handled by parent screen cleanup
}

void DualScreenManager::UpdateAnimationSource(const std::string& anim_path) {
    if (anim_path.empty()) {
        ESP_LOGE(TAG, "Received empty animation path.");
        return;
    }
    ESP_LOGI(TAG, "Updating animation source to: %s", anim_path.c_str());

    if (m_left_gif_obj) {
        lv_gif_set_src(m_left_gif_obj, anim_path.c_str());
    }
    if (m_right_gif_obj) {
        lv_gif_set_src(m_right_gif_obj, anim_path.c_str());
    }
}

void DualScreenManager::clear_disp(lv_display_t* disp) {
    if (!disp) return;
    lv_obj_t* screen_obj = lv_display_get_screen_active(disp);
    if (screen_obj) {
        lv_obj_clean(screen_obj);
        // After cleaning, we need to recreate the persistent GIF objects
        if (disp == m_left_disp) {
            m_left_gif_obj = lv_gif_create(screen_obj);
            lv_obj_align(m_left_gif_obj, LV_ALIGN_CENTER, 0, 0);
        } else if (disp == m_right_disp) {
            m_right_gif_obj = lv_gif_create(screen_obj);
            lv_obj_align(m_right_gif_obj, LV_ALIGN_CENTER, 0, 0);
        }
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

lv_obj_t* DualScreenManager::get_gif_obj(ScreenId screen) const {
    if (screen == SCREEN_LEFT) {
        return m_left_gif_obj;
    }
    if (screen == SCREEN_RIGHT) {
        return m_right_gif_obj;
    }
    // For SCREEN_BOTH, we consistently use the left as the primary
    return m_left_gif_obj;
}