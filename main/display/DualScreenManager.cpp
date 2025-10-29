#include "DualScreenManager.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "GC9A01_driver.hpp" // For mirror mode control

static const char* TAG = "DualScreenManager";

DualScreenManager::DualScreenManager() 
    : m_left_disp(get_left_screen_display()), 
      m_right_disp(get_right_screen_display()),
      m_left_gif_obj(nullptr),
      m_right_gif_obj(nullptr) {
    ESP_LOGI(TAG, "DualScreenManager constructed, displays acquired.");
    // LVGL objects will be created in init()
}

void DualScreenManager::init() {
    ESP_LOGI(TAG, "Initializing LVGL objects for DualScreenManager.");
    set_mirror_mode(false); // Default to independent screens

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

void DualScreenManager::UpdateAnimationSource(const AnimationPair& anim_pair) {
    if (!anim_pair.is_valid()) {
        ESP_LOGE(TAG, "Received invalid animation pair.");
        if (m_left_gif_obj) lv_obj_add_flag(m_left_gif_obj, LV_OBJ_FLAG_HIDDEN);
        if (m_right_gif_obj) lv_obj_add_flag(m_right_gif_obj, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    ESP_LOGD(TAG, "Updating animation source. Mirrored: %s", anim_pair.is_mirrored ? "true" : "false");

    // Set mirror mode based on the provided animation pair
    set_mirror_mode(anim_pair.is_mirrored);

    // Ensure the GIF objects are visible
    if (m_left_gif_obj) lv_obj_remove_flag(m_left_gif_obj, LV_OBJ_FLAG_HIDDEN);
    if (m_right_gif_obj) lv_obj_remove_flag(m_right_gif_obj, LV_OBJ_FLAG_HIDDEN);

    // We'll only set flags and prepare descriptors here. Actual lv_gif_set_src
    // and lv_gif_restart will be done once at the end so both screens start
    // as close to simultaneously as possible.
    bool update_left = false;
    bool update_right = false;

    // --- Prepare Left Screen ---
    if (m_left_gif_obj && anim_pair.left_anim.is_valid) {
        m_left_img_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
        m_left_img_dsc.header.w = 0;
        m_left_img_dsc.header.h = 0;
        m_left_img_dsc.header.cf = LV_COLOR_FORMAT_UNKNOWN;
        m_left_img_dsc.data_size = anim_pair.left_anim.size;
        m_left_img_dsc.data = anim_pair.left_anim.data;
        update_left = true;
    } else if (m_left_gif_obj) {
        lv_obj_add_flag(m_left_gif_obj, LV_OBJ_FLAG_HIDDEN);
        update_left = false;
    }

    // --- Prepare Right Screen ---
    if (m_right_gif_obj && anim_pair.right_anim.is_valid) {
        if (anim_pair.is_mirrored) {
            // In mirror mode, right screen uses the same descriptor as the left
            m_right_img_dsc = m_left_img_dsc;
        } else {
            // In independent mode, create a new descriptor for the right screen
            m_right_img_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
            m_right_img_dsc.header.w = 0;
            m_right_img_dsc.header.h = 0;
            m_right_img_dsc.header.cf = LV_COLOR_FORMAT_UNKNOWN;
            m_right_img_dsc.data_size = anim_pair.right_anim.size;
            m_right_img_dsc.data = anim_pair.right_anim.data;
        }
        update_right = true;
    } else if (m_right_gif_obj) {
        lv_obj_add_flag(m_right_gif_obj, LV_OBJ_FLAG_HIDDEN);
        update_right = false;
    }

    // --- Apply sources and restart GIFs together for best synchronization ---
    if (update_left && m_left_gif_obj) {
        lv_gif_set_src(m_left_gif_obj, &m_left_img_dsc);
    }
    if (update_right && m_right_gif_obj) {
        lv_gif_set_src(m_right_gif_obj, &m_right_img_dsc);
    }

    // Restart both in quick succession so playback starts nearly synced.
    if ((update_left || update_right)) {
        if (update_left && m_left_gif_obj) lv_gif_restart(m_left_gif_obj);
        if (update_right && m_right_gif_obj) lv_gif_restart(m_right_gif_obj);
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

    // Disabling mirror mode since we are breaking the synchronized state
    set_mirror_mode(false);

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