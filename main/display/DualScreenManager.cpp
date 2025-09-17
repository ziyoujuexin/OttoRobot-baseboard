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

    // 清理屏幕上的旧对象
    // lv_obj_clean(screen_obj) 在当前场景下会触发看门狗，原因可能与LVGL内部状态有关。
    // 改为手动删除所有子对象，这在功能上等同于清屏，但可以规避问题。
    for (int i = lv_obj_get_child_count(screen_obj) - 1; i >= 0; i--) {
        lv_obj_del(lv_obj_get_child(screen_obj, i));
    }

    // Create and display the new animation
    lv_obj_t* gif = lv_gif_create(screen_obj);
    lv_gif_set_src(gif, anim_path.c_str());
    if(lv_gif_is_loaded(gif)) {
        ESP_LOGI(TAG, "GIF loaded successfully from path: %s", anim_path.c_str());
    } else {
        ESP_LOGE(TAG, "Failed to load GIF from path: %s", anim_path.c_str());
    }
    lv_obj_align(gif, LV_ALIGN_CENTER, 0, 0);

    // ESP_LOGI(TAG, "Creating a simple red square to test rendering...");
    // lv_obj_t * obj = lv_obj_create(screen_obj);
    
    // // 设置为红色
    // lv_obj_set_style_bg_color(obj, lv_color_hex(0xFF0000), 0);
    
    // // 设置大小和位置
    //     // Diagnostic: Change size to fit in one buffer chunk to test LVGL's rendering logic.
    // // Original was 100x100, which requires chunking.
    // lv_obj_set_size(obj, 30, 30);
    // lv_obj_center(obj);
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
