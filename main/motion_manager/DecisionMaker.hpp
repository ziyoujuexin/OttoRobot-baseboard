#pragma once

#include "Motion_types.hpp"
#include <functional>

class MotionController; // Forward declaration

class DecisionMaker
{
public:
    DecisionMaker(MotionController& motion_controller);
    void start();
    void set_face_location(const FaceLocation& location);

private:
    void decision_maker_task();

    MotionController& m_motion_controller;
    FaceLocation m_last_face_location;
    // Add any state variables needed for decision making
    // e.g., bool m_is_walking;
};