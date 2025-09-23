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
    // if (!m_left_disp || !m_right_disp) {
    //     ESP_LOGE(TAG, "Failed to get display handles!");
    //     return;
    // }

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

void DualScreenManager::UpdateAnimationSource(const AnimationData& anim_data) {
    if (!anim_data.is_valid || !anim_data.data) {
        ESP_LOGE(TAG, "Received invalid animation data.");
        // Optionally, hide the GIF object if the data is invalid
        if (m_left_gif_obj) lv_obj_add_flag(m_left_gif_obj, LV_OBJ_FLAG_HIDDEN);
        if (m_right_gif_obj) lv_obj_add_flag(m_right_gif_obj, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    ESP_LOGI(TAG, "Updating animation source from memory at %p, size %d", anim_data.data, anim_data.size);

    // Ensure the GIF objects are visible
    if (m_left_gif_obj) lv_obj_remove_flag(m_left_gif_obj, LV_OBJ_FLAG_HIDDEN);
    if (m_right_gif_obj) lv_obj_remove_flag(m_right_gif_obj, LV_OBJ_FLAG_HIDDEN);

    // Create an image descriptor for the in-memory data
    // IMPORTANT: The `anim_data` (and the buffer it points to) MUST remain valid
    // for the lifetime of the GIF object, or until the source is changed.
    m_left_img_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    m_left_img_dsc.header.w = 0; // For GIF, width and height are read from the data
    m_left_img_dsc.header.h = 0;
    m_left_img_dsc.header.cf = LV_COLOR_FORMAT_UNKNOWN; // Color format is also from data
    m_left_img_dsc.data_size = anim_data.size;
    m_left_img_dsc.data = anim_data.data;

    // The right screen uses the same data source descriptor
    m_right_img_dsc = m_left_img_dsc;

    if (m_left_gif_obj) {
        lv_gif_set_src(m_left_gif_obj, &m_left_img_dsc);
        lv_gif_restart(m_left_gif_obj);
    }
    if (m_right_gif_obj) {
        lv_gif_set_src(m_right_gif_obj, &m_right_img_dsc);
        lv_gif_restart(m_right_gif_obj);
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