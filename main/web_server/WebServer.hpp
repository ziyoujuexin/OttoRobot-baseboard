#pragma once

#include "MotionController.hpp"
#include "motion_manager/ActionManager.hpp"
#include <memory>

// Forward declaration to avoid circular includes
class AnimationPlayer;

class WebServer {
public:
    WebServer(ActionManager& action_manager, MotionController& motion_controller, AnimationPlayer& animation_player);
    void start();

private:
    ActionManager& m_action_manager;
    MotionController& m_motion_controller;
    AnimationPlayer& m_animation_player;
};
