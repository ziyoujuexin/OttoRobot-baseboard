#include "SDCardAnimationProvider.h"
#include "esp_log.h"
#include <fstream> // For std::ifstream
#include <iterator> // For std::istreambuf_iterator

static const char* TAG = "SDCardProvider";

SDCardAnimationProvider::SDCardAnimationProvider(const std::string& base_path)
    : m_base_path(base_path) {}

AnimationData SDCardAnimationProvider::getAnimationData(const std::string& animation_name) {
    AnimationData anim_data;
    anim_data.name = animation_name;

    std::string path = m_base_path + "/" + animation_name + ".gif";

    // Use C++ streams to read the binary file. This works with ESP-IDF VFS.
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        ESP_LOGW(TAG, "Failed to open animation file: %s", path.c_str());
        return anim_data; // Return empty AnimationData
    }

    // Read the whole file into the vector
    anim_data.data = std::vector<uint8_t>((std::istreambuf_iterator<char>(file)),
                                           std::istreambuf_iterator<char>());

    if (anim_data.IsValid()) {
        ESP_LOGI(TAG, "Successfully read %zu bytes for animation '%s' from %s", 
                 anim_data.data.size(), animation_name.c_str(), path.c_str());
    } else {
        ESP_LOGW(TAG, "Animation file is empty: %s", path.c_str());
    }

    return anim_data;
}
