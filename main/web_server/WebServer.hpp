#pragma once

#include "MotionController.hpp"
#include "motion_manager/ActionManager.hpp"
#include <memory>

class WebServer {
public:
    WebServer(ActionManager& action_manager, MotionController& motion_controller);
    void start();

private:
    ActionManager& m_action_manager;
    MotionController& m_motion_controller;
};
