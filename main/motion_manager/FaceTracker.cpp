#include "FaceTracker.hpp"
#include <cmath>
#include <algorithm>

// Constants from the original implementation
static constexpr float KP = 0.08f;
static constexpr float KD = 0.04f;
static constexpr int DEADZONE_PIXELS = 5;
static constexpr float DELTA_LIMIT = 10.0f;
static constexpr int SCREEN_CENTER_X = 640 / 2;
static constexpr int SCREEN_CENTER_Y = 480 / 2;

FaceTracker::FaceTracker() {
    reset();
}

void FaceTracker::reset() {
    m_pid_pan_error_last = 0;
    m_pid_tilt_error_last = 0;
    m_pan_offset = 0.0f;
    m_tilt_offset = 0.0f;
    m_is_active = false;
    m_current_face_location = {0, 0, 0, 0, false};
}

void FaceTracker::set_active(bool active) {
    if (m_is_active && !active) { // If turning off
        // Reset PID errors when stopping to prevent jumps when restarting
        m_pid_pan_error_last = 0;
        m_pid_tilt_error_last = 0;
    }
    m_is_active = active;
}

bool FaceTracker::is_active() const {
    return m_is_active;
}

void FaceTracker::set_face_location(const FaceLocation& location) {
    m_current_face_location = location;
}

float FaceTracker::get_pan_offset() const {
    return m_pan_offset;
}

HeadPose FaceTracker::update() {
    if (!m_is_active || !m_current_face_location.detected) {
        // If not active or no face detected, return current offsets without change
        return {m_pan_offset, m_tilt_offset};
    }

    // --- PD Controller Logic (migrated from MotionController) ---
    int error_pan = SCREEN_CENTER_X - (m_current_face_location.x + m_current_face_location.w / 2);
    if (std::abs(error_pan) < DEADZONE_PIXELS) { error_pan = 0; }
    float derivative_pan = error_pan - m_pid_pan_error_last;
    float output_pan = KP * error_pan + KD * derivative_pan;
    m_pid_pan_error_last = error_pan;

    int error_tilt = (m_current_face_location.y + m_current_face_location.h / 2) - SCREEN_CENTER_Y;
    if (std::abs(error_tilt) < DEADZONE_PIXELS) { error_tilt = 0; }
    float derivative_tilt = error_tilt - m_pid_tilt_error_last;
    float output_tilt = KP * 0.6f * error_tilt + KD * derivative_tilt;
    m_pid_tilt_error_last = error_tilt;

    if (!std::isfinite(output_pan)) { output_pan = 0.0f; }
    if (!std::isfinite(output_tilt)) { output_tilt = 0.0f; }

    // Limit the change per update cycle
    if (output_pan > DELTA_LIMIT)  { output_pan = DELTA_LIMIT; }
    if (output_pan < -DELTA_LIMIT) { output_pan = -DELTA_LIMIT; }
    if (output_tilt > DELTA_LIMIT * 0.6f) { output_tilt = DELTA_LIMIT * 0.6f; }
    if (output_tilt < -DELTA_LIMIT * 0.6f){ output_tilt = -DELTA_LIMIT * 0.6f; }

    m_pan_offset += output_pan;
    m_tilt_offset += output_tilt;

    // Clamp the final offsets
    m_pan_offset = std::max(-70.0f, std::min(70.0f, m_pan_offset));
    m_tilt_offset = std::max(-40.0f, std::min(40.0f, m_tilt_offset));

    return {m_pan_offset, m_tilt_offset};
}
