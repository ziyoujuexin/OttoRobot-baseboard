#include "SDCardAnimationProvider.h"
#include "esp_log.h"
#include <sys/stat.h> // For stat

static const char* TAG = "SDCardProvider";

SDCardAnimationProvider::SDCardAnimationProvider(const std::string& base_path)
    : m_base_path(base_path) {}

std::string SDCardAnimationProvider::getAnimationPath(const std::string& animation_name) {
    std::string vfs_path = m_base_path + "/" + animation_name + ".gif";

    // Check if the file exists using stat, which works with the VFS
    struct stat st;
    if (stat(vfs_path.c_str(), &st) == 0) {
        ESP_LOGI(TAG, "Found animation '%s' at VFS path: %s", animation_name.c_str(), vfs_path.c_str());
        
        // Convert VFS path to LVGL path with drive letter. e.g., "/sdcard/anim.gif" -> "S:/anim.gif"
        std::string lvgl_path = vfs_path;
        std::string prefix_to_replace = "/sdcard";
        size_t pos = lvgl_path.find(prefix_to_replace);
        if (pos == 0) {
            lvgl_path.replace(pos, prefix_to_replace.length(), "S:");
        }

        ESP_LOGI(TAG, "Returning LVGL path: %s", lvgl_path.c_str());
        return lvgl_path;
    } else {
        ESP_LOGW(TAG, "Animation '%s' not found at VFS path: %s", animation_name.c_str(), vfs_path.c_str());
        return ""; // Return empty string if not found
    }
}
