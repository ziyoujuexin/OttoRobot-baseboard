#pragma once

#include "motion_manager/Motion_types.hpp"
#include "motion_manager/MotionStorage.hpp"
#include <memory>
#include <string>
#include <vector>
#include <map>

class ActionManager {
public:
    ActionManager();
    ~ActionManager();

    void init();

    // Action data access
    bool get_action(const std::string& name, RegisteredAction& action) const;

    // NVS Storage Interface
    bool delete_action_from_nvs(const std::string& action_name);
    bool delete_group_from_nvs(const std::string& group_name);
    std::vector<std::string> list_actions_from_nvs();
    std::vector<std::string> list_groups_from_nvs();

    // Real-time Gait Tuning & API methods
    bool update_action_properties(const std::string& action_name, bool is_atomic, uint32_t default_steps, uint32_t gait_period_ms);
    bool tune_gait_parameter(const std::string& action_name, int servo_index, const std::string& param_type, float value);
    bool save_action_to_nvs(const std::string& action_name);
    std::string get_action_params_json(const std::string& action_name);

    // keep this register function for public before moction is completed, change to private when release
    void register_default_actions();
private:

    void print_action_details(const RegisteredAction &action);

    std::unique_ptr<MotionStorage> m_storage;
    std::map<std::string, RegisteredAction> m_action_cache;
    std::map<std::string, RegisteredGroup> m_group_cache;
};
