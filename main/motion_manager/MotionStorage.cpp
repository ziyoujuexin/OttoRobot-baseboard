#include "MotionStorage.hpp"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "MotionStorage";

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

    char key[NVS_KEY_NAME_MAX_SIZE];
    snprintf(key, sizeof(key), "%s", action.name);

    err = nvs_set_blob(handle, key, &action, sizeof(RegisteredAction));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save action '%s'. Error: %s", key, esp_err_to_name(err));
    } else {
        err = nvs_commit(handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit NVS changes. Error: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Action '%s' saved successfully.", key);
        }
    }

    nvs_close(handle);
    return err == ESP_OK;
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

    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to load action '%s'. Error: %s", name, esp_err_to_name(err));
    }
    
    return err == ESP_OK;
}

bool MotionStorage::delete_action(const char* name) {
    if (!m_initialized) return false;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(m_nvs_namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) return false;

    char key[NVS_KEY_NAME_MAX_SIZE];
    snprintf(key, sizeof(key), "%s", name);

    err = nvs_erase_key(handle, key);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete action '%s'. Error: %s", key, esp_err_to_name(err));
    } else {
        err = nvs_commit(handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit deletion for '%s'. Error: %s", key, esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Action '%s' deleted successfully.", key);
        }
    }

    nvs_close(handle);
    return err == ESP_OK;
}

bool MotionStorage::list_actions(std::vector<std::string>& action_names) {
    action_names.clear();
    if (!m_initialized) return false;

    nvs_iterator_t it = nullptr; // 初始化为 nullptr
    esp_err_t err = nvs_entry_find(NVS_DEFAULT_PART_NAME, m_nvs_namespace, NVS_TYPE_BLOB, &it);
    if (err == ESP_ERR_NVS_NOT_FOUND || it == nullptr) {
        ESP_LOGI(TAG, "No actions found in NVS namespace '%s'.", m_nvs_namespace);
        return true;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error finding NVS entries: %s", esp_err_to_name(err));
        return false;
    }

    const std::string prefix = "";
    while (it != nullptr) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);
        if (strncmp(info.key, prefix.c_str(), prefix.length()) == 0) {
            action_names.push_back(info.key + prefix.length());
        }
        err = nvs_entry_next(&it); // 传递迭代器指针
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error iterating NVS entries: %s", esp_err_to_name(err));
            break;
        }
    }
    nvs_release_iterator(it);
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

    char key[NVS_KEY_NAME_MAX_SIZE];
    snprintf(key, sizeof(key), "%s", group.name);

    err = nvs_set_blob(handle, key, &group, sizeof(RegisteredGroup));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save group '%s'. Error: %s", key, esp_err_to_name(err));
    } else {
        err = nvs_commit(handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit NVS for group. Error: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Group '%s' saved successfully.", key);
        }
    }

    nvs_close(handle);
    return err == ESP_OK;
}

bool MotionStorage::load_group(const char* name, RegisteredGroup& group) {
    if (!m_initialized) return false;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(m_nvs_namespace, NVS_READONLY, &handle);
    if (err != ESP_OK) return false;

    char key[NVS_KEY_NAME_MAX_SIZE];
    snprintf(key, sizeof(key), "%s", name);

    size_t required_size = sizeof(RegisteredGroup);
    err = nvs_get_blob(handle, key, &group, &required_size);
    nvs_close(handle);

    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to load group '%s'. Error: %s", key, esp_err_to_name(err));
    }

    return err == ESP_OK;
}

bool MotionStorage::delete_group(const char* name) {
    if (!m_initialized) return false;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(m_nvs_namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) return false;

    char key[NVS_KEY_NAME_MAX_SIZE];
    snprintf(key, sizeof(key), "%s", name);

    err = nvs_erase_key(handle, key);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete group '%s'. Error: %s", key, esp_err_to_name(err));
    } else {
        err = nvs_commit(handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit deletion for group '%s'. Error: %s", key, esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Group '%s' deleted successfully.", key);
        }
    }
    
    nvs_close(handle);
    return err == ESP_OK;
}

bool MotionStorage::list_groups(std::vector<std::string>& group_names) {
    group_names.clear();
    if (!m_initialized) return false;

    nvs_iterator_t it = nullptr; // 初始化为 nullptr
    esp_err_t err = nvs_entry_find(NVS_DEFAULT_PART_NAME, m_nvs_namespace, NVS_TYPE_BLOB, &it);
    if (err == ESP_ERR_NVS_NOT_FOUND || it == nullptr) {
        ESP_LOGI(TAG, "No groups found in NVS namespace '%s'.", m_nvs_namespace);
        return true;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error finding NVS entries: %s", esp_err_to_name(err));
        return false;
    }

    const std::string prefix = "";
    while (it != nullptr) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);
        if (strncmp(info.key, prefix.c_str(), prefix.length()) == 0) {
            group_names.push_back(info.key + prefix.length());
        }
        err = nvs_entry_next(&it); // 传递迭代器指针
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error iterating NVS entries: %s", esp_err_to_name(err));
            break;
        }
    }
    nvs_release_iterator(it);
    return true;
}
