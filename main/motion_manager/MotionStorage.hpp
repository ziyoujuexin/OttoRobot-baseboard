#pragma once

#include "Motion_types.hpp"
#include <vector>
#include <string>

class MotionStorage {
public:
    MotionStorage(const char* nvs_namespace = "motion_db");
    ~MotionStorage();

    // 初始化NVS
    bool init();

    // --- Action Management ---
    bool save_action(const RegisteredAction& action);
    bool load_action(const char* name, RegisteredAction& action);
    bool delete_action(const char* name);
    bool list_actions(std::vector<std::string>& action_names);

    // --- Group Management ---
    bool save_group(const RegisteredGroup& group);
    bool load_group(const char* name, RegisteredGroup& group);
    bool delete_group(const char* name);
    bool list_groups(std::vector<std::string>& group_names);

private:
    const char* m_nvs_namespace;
    bool m_initialized = false;
};