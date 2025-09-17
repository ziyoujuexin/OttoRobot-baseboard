#include "DualScreenManager.h"
#include "esp_log.h"

static const char* TAG = "DualScreenManager";

DualScreenManager::DualScreenManager() 
    : m_left_disp(get_left_screen_display()), 
      m_right_disp(get_right_screen_display()),
      m_left_gif_obj(nullptr),
      m_right_gif_obj(nullptr) {
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

    lv_obj_t** gif_obj_ptr = (disp == m_left_disp) ? &m_left_gif_obj : &m_right_gif_obj;

    // If the gif object for this screen doesn't exist yet, create it.
    if (*gif_obj_ptr == nullptr) {
        ESP_LOGI(TAG, "Creating new GIF object for the screen.");
        *gif_obj_ptr = lv_gif_create(screen_obj);
        lv_obj_align(*gif_obj_ptr, LV_ALIGN_CENTER, 0, 0);
    } else {
        ESP_LOGI(TAG, "Reusing existing GIF object for the screen.");
    }
    // for (int i = lv_obj_get_child_count(screen_obj) - 1; i >= 0; i--) {
    //     lv_obj_del(lv_obj_get_child(screen_obj, i));
    // }
    // Set the source for the new or existing gif object.
    lv_gif_set_src(*gif_obj_ptr, anim_path.c_str());
    lv_gif_set_loop_count(*gif_obj_ptr, 5);
    lv_gif_resume(*gif_obj_ptr);
    // lv_gif_restart(*gif_obj_ptr);

    if(lv_gif_is_loaded(*gif_obj_ptr)) {
        ESP_LOGI(TAG, "GIF loaded successfully from path: %s", anim_path.c_str());
    } else {
        ESP_LOGE(TAG, "Failed to load GIF from path: %s", anim_path.c_str());
    }
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