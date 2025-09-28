#pragma once

#include "MotionController.hpp"
#include "motion_manager/ActionManager.hpp"
#include "freertos/queue.h" // Required for QueueHandle_t
#include <memory>

class WebServer {
public:
    WebServer(ActionManager& action_manager, MotionController& motion_controller, QueueHandle_t ui_command_queue);
    void start();

private:
    ActionManager& m_action_manager;
    MotionController& m_motion_controller;
    QueueHandle_t m_ui_command_queue;
};
