#include "MotionStorage.hpp"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "MotionStorage";

// Helper to manage the index of keys
static esp_err_t update_key_index(nvs_handle_t handle, const char* index_key, const char* item_key, bool add) {
    // Implementation for adding/removing keys from an index will be added in a future step
    // For now, we will just log the operation
    if (add) {
        ESP_LOGI(TAG, "Adding key '%s' to index '%s' (not implemented yet)", item_key, index_key);
    } else {
        ESP_LOGI(TAG, "Removing key '%s' from index '%s' (not implemented yet)", item_key, index_key);
    }
    return ESP_OK;
}


MotionStorage::MotionStorage(const char* nvs_namespace) : m_nvs_namespace(nvs_namespace) {}

MotionStorage::~MotionStorage() {}

bool MotionStorage::init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition contains no free pages or has a new version, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS Flash Initialized.");
    m_initialized = true;
    return true;
}

// --- Action Management --- 
bool MotionStorage::save_action(const RegisteredAction& action) {
    if (!m_initialized) return false;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(m_nvs_namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return false;
    }

    err = nvs_set_blob(handle, action.name, &action, sizeof(RegisteredAction));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save action '%s'. Error: %s", action.name, esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }

    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS changes. Error: %s", esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Action '%s' saved successfully.", action.name);
    return true;
}

bool MotionStorage::load_action(const char* name, RegisteredAction& action) {
    if (!m_initialized) return false;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(m_nvs_namespace, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return false;
    }

    size_t required_size = sizeof(RegisteredAction);
    err = nvs_get_blob(handle, name, &action, &required_size);
    nvs_close(handle);

    if (err != ESP_OK) {
        if (err != ESP_ERR_NVS_NOT_FOUND) {
             ESP_LOGE(TAG, "Failed to load action '%s'. Error: %s", name, esp_err_to_name(err));
        }
        return false;
    }
    
    return true;
}

bool MotionStorage::delete_action(const char* name) {
    if (!m_initialized) return false;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(m_nvs_namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) return false;

    err = nvs_erase_key(handle, name);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete action '%s'. Error: %s", name, esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }

    err = nvs_commit(handle);
    nvs_close(handle);
    return err == ESP_OK;
}

bool MotionStorage::list_actions(std::vector<std::string>& action_names) {
    // This is a simplified implementation. A robust solution would require managing an index.
    // For now, we cannot list keys directly from NVS. Returning true with an empty list.
    ESP_LOGW(TAG, "list_actions is not fully implemented and will return an empty list.");
    action_names.clear();
    return true;
}

// --- Group Management ---
bool MotionStorage::save_group(const RegisteredGroup& group) {
    if (!m_initialized) return false;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(m_nvs_namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle for group!", esp_err_to_name(err));
        return false;
    }

    err = nvs_set_blob(handle, group.name, &group, sizeof(RegisteredGroup));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save group '%s'. Error: %s", group.name, esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }

    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS for group. Error: %s", esp_err_to_name(err));
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Group '%s' saved successfully.", group.name);
    return err == ESP_OK;
}

bool MotionStorage::load_group(const char* name, RegisteredGroup& group) {
    if (!m_initialized) return false;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(m_nvs_namespace, NVS_READONLY, &handle);
    if (err != ESP_OK) return false;

    size_t required_size = sizeof(RegisteredGroup);
    err = nvs_get_blob(handle, name, &group, &required_size);
    nvs_close(handle);

    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to load group '%s'. Error: %s", name, esp_err_to_name(err));
    }

    return err == ESP_OK;
}

bool MotionStorage::delete_group(const char* name) {
    if (!m_initialized) return false;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(m_nvs_namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) return false;

    err = nvs_erase_key(handle, name);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete group '%s'. Error: %s", name, esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }

    err = nvs_commit(handle);
    nvs_close(handle);
    return err == ESP_OK;
}

bool MotionStorage::list_groups(std::vector<std::string>& group_names) {
    ESP_LOGW(TAG, "list_groups is not fully implemented and will return an empty list.");
    group_names.clear();
    return true;
}