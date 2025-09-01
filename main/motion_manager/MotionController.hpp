#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "driver/servo.hpp"
#include "config.h"
#include "motion_manager/Motion_types.hpp"
#include "motion_manager/ActionManager.hpp"
#include "motion_manager/DecisionMaker.hpp" // Include the new header
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <atomic>
#include <vector>
#include <string>

class DecisionMaker; // Forward declaration

// Defines the mode for the home() method
enum class HomeMode {
    All,        // Home all servos
    Whitelist,  // Home only servos in the provided list
    Blacklist   // Home all servos except those in the provided list
};

class MotionController {
public:
    explicit MotionController(Servo& servo_driver, ActionManager& action_manager);
    ~MotionController();
    void init();
    bool queue_command(const motion_command_t& cmd);
    void set_single_servo(uint8_t channel, uint8_t angle);
    void home(HomeMode mode = HomeMode::All, const std::vector<ServoChannel>& channels = {});
    bool is_body_moving() const;
    bool queue_face_location(const FaceLocation& face_loc);
    DecisionMaker* get_decision_maker() const;
    bool is_face_tracking_active() const;

    motion_command_t get_current_command();
    bool is_idle() {
        return is_active == false;
    }

private:
    Servo& m_servo_driver; 
    ActionManager& m_action_manager;
    QueueHandle_t m_motion_queue; 
    std::map<uint8_t, std::string> m_gait_command_map; // Map command code to action name

    bool is_active = false;
    SemaphoreHandle_t m_actions_mutex;
    mutable std::vector<RegisteredAction> m_active_actions;
    // ---

    // Mapping from logical joint to physical servo channel
    uint8_t m_joint_channel_map[GAIT_JOINT_COUNT];

    std::atomic<bool> m_interrupt_flag; // Used for global STOP

    // --- New Face Location Queue ---
    QueueHandle_t m_face_location_queue;

    // --- Decision Maker ---
    std::unique_ptr<DecisionMaker> m_decision_maker;

    void init_joint_channel_map();

    // --- Task Declarations ---
    void motion_engine_task(); // Renamed to dispatcher task
    void motion_mixer_task();  // The new mixer task
    void face_tracking_task(); // New task for face tracking

    // --- Face Tracking Members (some moved to local in task) ---
    float m_pid_pan_error_last;
    float m_pid_tilt_error_last;
    float m_pan_offset;  // Current pan offset for head tracking
    float m_tilt_offset; // Current tilt offset for head tracking
    bool m_increment_was_limited_last_cycle;
    ActionInstance m_head_tracking_action;
    std::atomic<bool> m_is_tracking_active;
    int64_t m_last_tracking_turn_end_time;
    std::atomic<bool> m_is_head_frozen;
    std::atomic<bool> m_is_executed{false};

private:

    // --- Task Wrappers ---
    static void start_task_wrapper(void* _this) {
        static_cast<MotionController*>(_this)->motion_engine_task();
    }
    static void start_mixer_task_wrapper(void* _this) {
        static_cast<MotionController*>(_this)->motion_mixer_task();
    }
    static void start_face_tracking_task_wrapper(void* _this) {
        static_cast<MotionController*>(_this)->face_tracking_task();
    }
};