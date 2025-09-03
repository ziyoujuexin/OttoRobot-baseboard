#pragma once

#include "motion_manager/Motion_types.hpp"

// A simple struct to hold the calculated head angles.
// These represent the offset from the home position (90 degrees).
struct HeadPose {
    float pan_offset;
    float tilt_offset;
};

class FaceTracker {
public:
    FaceTracker();

    // Update the tracker with a new face location
    void set_face_location(const FaceLocation& location);

    // Calculate the required head adjustment based on current state
    HeadPose update();

    // Activate or deactivate the tracking logic
    void set_active(bool active);
    bool is_active() const;

    // Reset the internal state of the tracker (e.g., PID errors)
    void reset();

    // Gets the current pan offset, useful for DecisionMaker
    float get_pan_offset() const;

private:
    // PID and state variables moved from MotionController
    FaceLocation m_current_face_location;
    float m_pid_pan_error_last;
    float m_pid_tilt_error_last;
    float m_pan_offset;
    float m_tilt_offset;
    bool m_is_active;
};
