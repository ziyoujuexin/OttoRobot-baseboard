#include "SDCardAnimationProvider.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <cstdio>      // For FILE, fopen, etc.
#include <sys/stat.h>  // For stat
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "SDCardProvider";

// Helper function to load a single file into a new PSRAM buffer
static AnimationData load_file_to_psram(const std::string& vfs_path) {
    AnimationData anim_data;

    FILE* f = fopen(vfs_path.c_str(), "rb");
    if (!f) {
        // This is not an error, just means the file doesn't exist.
        // ESP_LOGD(TAG, "File not found at VFS path: %s", vfs_path.c_str());
        return anim_data; // Return invalid data
    }

    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (file_size <= 0) {
        ESP_LOGE(TAG, "File is empty or invalid size for %s", vfs_path.c_str());
        fclose(f);
        return anim_data;
    }

    uint8_t* buffer = (uint8_t*)heap_caps_malloc(file_size, MALLOC_CAP_SPIRAM);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate %ld bytes in PSRAM for %s", file_size, vfs_path.c_str());
        fclose(f);
        return anim_data;
    }

    const size_t chunkSize = 16 * 1024; // 16KB chunks
    size_t bytes_read = 0;
    while (bytes_read < (size_t)file_size) {
        size_t bytes_to_read = (size_t)file_size - bytes_read;
        if (bytes_to_read > chunkSize) {
            bytes_to_read = chunkSize;
        }

        size_t chunk_bytes_read = fread(buffer + bytes_read, 1, bytes_to_read, f);

        if (chunk_bytes_read != bytes_to_read) {
            ESP_LOGE(TAG, "File read error. Expected %zu, got %zu", bytes_to_read, chunk_bytes_read);
            fclose(f);
            heap_caps_free(buffer);
            return anim_data; // Return invalid data
        }

        bytes_read += chunk_bytes_read;

        // Yield to other tasks to prevent WDT timeout
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    // size_t bytes_read = fread(buffer, 1, file_size, f);
    fclose(f);

    if (bytes_read != (size_t)file_size) {
        ESP_LOGE(TAG, "Failed to read the full file. Expected %ld, got %zu for %s", file_size, bytes_read, vfs_path.c_str());
        heap_caps_free(buffer);
        return anim_data;
    }

    ESP_LOGI(TAG, "Successfully loaded '%s' (%d bytes) into PSRAM at %p", vfs_path.c_str(), bytes_read, buffer);
    anim_data.data = buffer;
    anim_data.size = bytes_read;
    anim_data.is_valid = true;
    return anim_data;
}

SDCardAnimationProvider::SDCardAnimationProvider(const std::string& base_path)
    : m_base_path(base_path) {}

AnimationPair SDCardAnimationProvider::getAnimationData(const std::string& animation_name) {
    AnimationPair anim_pair;

    // Sanitize the input to get a clean base_name, stripping both prefixes and suffix
    std::string base_name = animation_name;
    if (base_name.length() >= 4 && base_name.substr(base_name.length() - 4) == ".gif") {
        base_name = base_name.substr(0, base_name.length() - 4);
    }
    if (base_name.rfind("[L]", 0) == 0) { // starts with [L]
        base_name = base_name.substr(3);
    } else if (base_name.rfind("[R]", 0) == 0) { // starts with [R]
        base_name = base_name.substr(3);
    }

    // 1. Try to find a prefixed pair first for independent display
    std::string left_path = m_base_path + "/[L]" + base_name + ".gif";
    std::string right_path = m_base_path + "/[R]" + base_name + ".gif";

    AnimationData left_anim = load_file_to_psram(left_path);
    AnimationData right_anim = load_file_to_psram(right_path);

    if (left_anim.is_valid && right_anim.is_valid) {
        ESP_LOGI(TAG, "Found animation pair for '%s'. Using independent mode.", base_name.c_str());
        anim_pair.left_anim = left_anim;
        anim_pair.right_anim = right_anim;
        anim_pair.is_mirrored = false;
        return anim_pair;
    } else {
        // If we found one but not the other, release the one we found to avoid memory leaks
        if (left_anim.is_valid) releaseAnimationData(left_anim);
        if (right_anim.is_valid) releaseAnimationData(right_anim);
    }

    // 2. Fallback to finding a single, non-prefixed file for mirrored display
    std::string mirror_path = m_base_path + "/" + base_name + ".gif";
    AnimationData mirror_anim = load_file_to_psram(mirror_path);

    if (mirror_anim.is_valid) {
        ESP_LOGI(TAG, "Found single animation for '%s'. Using mirror mode.", base_name.c_str());
        anim_pair.left_anim = mirror_anim; // Both point to the same data
        anim_pair.right_anim = mirror_anim;
        anim_pair.is_mirrored = true;
        return anim_pair;
    }

    ESP_LOGW(TAG, "No animation found for name: %s", base_name.c_str());
    return anim_pair; // Return invalid pair
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
