#include "SDCardAnimationProvider.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <cstdio>      // For FILE, fopen, etc.
#include <sys/stat.h>  // For stat

static const char* TAG = "SDCardProvider";

SDCardAnimationProvider::SDCardAnimationProvider(const std::string& base_path)
    : m_base_path(base_path) {}

AnimationData SDCardAnimationProvider::getAnimationData(const std::string& animation_name) {
    AnimationData anim_data;
    std::string vfs_path;
    // Check if animation_name already ends with .gif
    if (animation_name.length() >= 4 && animation_name.substr(animation_name.length() - 4) == ".gif") {
        vfs_path = m_base_path + "/" + animation_name;
    } else {
        vfs_path = m_base_path + "/" + animation_name + ".gif";
    }

    // 1. Open the file
    FILE* f = fopen(vfs_path.c_str(), "rb");
    if (!f) {
        ESP_LOGW(TAG, "Animation file not found at VFS path: %s", vfs_path.c_str());
        return anim_data; // Return invalid data
    }

    // 2. Get file size
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (file_size <= 0) {
        ESP_LOGE(TAG, "File is empty or invalid size for %s", vfs_path.c_str());
        fclose(f);
        return anim_data;
    }

    // 3. Allocate memory in PSRAM
    // MALLOC_CAP_SPIRAM ensures the memory is allocated from PSRAM
    uint8_t* buffer = (uint8_t*)heap_caps_malloc(file_size, MALLOC_CAP_SPIRAM);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate %ld bytes in PSRAM for animation %s", file_size, animation_name.c_str());
        fclose(f);
        return anim_data;
    }

    // 4. Read the entire file into the buffer
    size_t bytes_read = fread(buffer, 1, file_size, f);
    fclose(f); // Close the file immediately after reading

    if (bytes_read != (size_t)file_size) {
        ESP_LOGE(TAG, "Failed to read the full animation file. Expected %ld, got %d", file_size, bytes_read);
        heap_caps_free(buffer); // Clean up allocated memory on failure
        return anim_data;
    }

    // 5. Populate the struct and return
    ESP_LOGI(TAG, "Successfully loaded animation '%s' (%d bytes) into PSRAM at %p",
             animation_name.c_str(), bytes_read, buffer);
             
    anim_data.data = buffer;
    anim_data.size = bytes_read;
    anim_data.is_valid = true;

    return anim_data;
}

void SDCardAnimationProvider::releaseAnimationData(AnimationData& anim_data) {
    if (anim_data.data && anim_data.is_valid) {
        ESP_LOGI(TAG, "Releasing %d bytes of PSRAM at %p", anim_data.size, anim_data.data);
        heap_caps_free((void*)anim_data.data);
    }
    // Invalidate the struct to prevent double-free
    anim_data.data = nullptr;
    anim_data.size = 0;
    anim_data.is_valid = false;
}
