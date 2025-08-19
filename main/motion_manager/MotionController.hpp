#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/servo.hpp"
#include "config.h"
#include "motion_manager/Motion_types.hpp"
#include "motion_manager/ActionManager.hpp"
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <atomic>

class MotionController {
public:
    explicit MotionController(Servo& servo_driver, ActionManager& action_manager);
    ~MotionController();
    void init();
    bool queue_command(const motion_command_t& cmd);
    void set_single_servo(uint8_t channel, uint8_t angle);

private:
    Servo& m_servo_driver; 
    ActionManager& m_action_manager;
    QueueHandle_t m_motion_queue; 
    std::map<uint8_t, std::string> m_gait_command_map; // Map command code to action name

    // --- New members for Motion Mixer ---
    SemaphoreHandle_t m_actions_mutex;
    std::vector<RegisteredAction> m_active_actions;
    // ---

    // Mapping from logical joint to physical servo channel
    uint8_t m_joint_channel_map[GAIT_JOINT_COUNT];

    std::atomic<bool> m_interrupt_flag; // Used for global STOP

    void init_joint_channel_map();

    // --- Task Declarations ---
    void motion_engine_task(); // Renamed to dispatcher task
    void motion_mixer_task();  // The new mixer task
    void face_tracking_task(); // New task for face tracking

    void home();

    // --- Face Tracking Members ---
    SemaphoreHandle_t m_face_data_mutex;
    FaceLocation m_last_face_location;
    float m_pid_pan_error_last;
    float m_pid_tilt_error_last;
    bool m_increment_was_limited_last_cycle;

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
