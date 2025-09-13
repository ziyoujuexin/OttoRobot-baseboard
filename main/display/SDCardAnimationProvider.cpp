#include "SDCardAnimationProvider.h"
#include "esp_log.h"
#include <fstream> // For std::ifstream
#include <iterator> // For std::istreambuf_iterator
#include <dirent.h>

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
        listAnimations(); // List available animations for debugging
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

void SDCardAnimationProvider::listAnimations() {
    ESP_LOGI(TAG, "Listing contents of directory: %s", m_base_path.c_str());

    // 打开目录
    DIR* dir = opendir(m_base_path.c_str());
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory: %s", m_base_path.c_str());
        return;
    }

    // 读取目录中的条目
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        // dirent 结构体包含 d_name (名称) 和 d_type (类型)
        // d_type 可以是 DT_REG (常规文件) 或 DT_DIR (目录)
        
        // 忽略 . (当前目录) 和 .. (上级目录)
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        if (entry->d_type == DT_DIR) {
            ESP_LOGI(TAG, "  [Dir]  %s", entry->d_name);
        } else if (entry->d_type == DT_REG) {
            ESP_LOGI(TAG, "  [File] %s", entry->d_name);
        } else {
            ESP_LOGI(TAG, "  [Other] %s", entry->d_name);
        }
    }

    // 关闭目录，释放资源
    closedir(dir);
}