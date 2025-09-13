#include "SDCardAnimationProvider.h"
#include "esp_log.h"
#include <sys/stat.h> // For stat

static const char* TAG = "SDCardProvider";

SDCardAnimationProvider::SDCardAnimationProvider(const std::string& base_path)
    : m_base_path(base_path) {}

std::string SDCardAnimationProvider::getAnimationPath(const std::string& animation_name) {
    std::string path = m_base_path + "/" + animation_name + ".gif";

    // Check if the file exists using stat, which works with the VFS
    struct stat st;
    if (stat(path.c_str(), &st) == 0) {
        ESP_LOGI(TAG, "Found animation '%s' at path: %s", animation_name.c_str(), path.c_str());
        return path;
    } else {
        ESP_LOGW(TAG, "Animation '%s' not found at path: %s", animation_name.c_str(), path.c_str());
        return ""; // Return empty string if not found
    }
}
