#pragma once

#include "MotionController.hpp"
#include "motion_manager/ActionManager.hpp"
#include <memory>

// Forward declaration to avoid circular includes
class AnimationPlayer;

class WebServer {
public:
    WebServer(AnimationPlayer& animation_player);
    void start();

private:
    AnimationPlayer& m_animation_player;
};
