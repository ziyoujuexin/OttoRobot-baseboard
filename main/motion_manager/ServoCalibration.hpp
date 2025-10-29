#pragma once

#include "config.h"
#include <array>

namespace ServoCalibration {

// Neutral position offsets (trims). The value to ADD to 90 to get the calibrated home.
// A value of -10 means the calibrated home is 80 degrees.
const std::array<float, static_cast<size_t>(ServoChannel::SERVO_COUNT)> trims = {{
    0.0f,  // 0: LEFT_EAR_LIFT
    0.0f,  // 1: LEFT_EAR_SWING
    0.0f,  // 2: RIGHT_EAR_LIFT
    0.0f,  // 3: RIGHT_EAR_SWING
    -30.0f, // 4: HEAD_TILT (90 - 30 = 60)
    -45.0f, // 5: HEAD_PAN (90 - 45 = 45)
    0.0f,  // 6: RIGHT_ARM_SWING
    0.0f,  // 7: LEFT_ARM_LIFT
    0.0f,  // 8: LEFT_ARM_SWING
    0.0f,  // 9: RIGHT_ARM_LIFT
    0.0f,   // 10: LEFT_LEG_ROTATE (90 + 0 = 90)
    10.0f, // 11: LEFT_ANKLE_LIFT (90 + 10 = 100)
    0.0f,  // 12: RIGHT_LEG_ROTATE
    0.0f   // 13: RIGHT_ANKLE_LIFT
}};

// Min and Max angle limits for each servo
struct AngleLimits {
    float min;
    float max;
};

const std::array<AngleLimits, static_cast<size_t>(ServoChannel::SERVO_COUNT)> limits = {{
    {0.0f, 180.0f},   // 0: LEFT_EAR_LIFT
    {0.0f, 180.0f},   // 1: LEFT_EAR_SWING
    {0.0f, 180.0f},   // 2: RIGHT_EAR_LIFT
    {0.0f, 180.0f},   // 3: RIGHT_EAR_SWING
    {0.0f, 90.0f},   // 4: HEAD_TILT
    {0.0f, 180.0f},   // 5: HEAD_PAN
    {0.0f, 180.0f},   // 6: RIGHT_ARM_SWING
    {50.0f, 80.0f},   // 7: LEFT_ARM_LIFT
    {0.0f, 180.0f},   // 8: LEFT_ARM_SWING
    {100.0f, 130.0f}, // 9: RIGHT_ARM_LIFT
    {0.0f, 180.0f},   // 10: LEFT_LEG_ROTATE
    {60.0f, 170.0f},  // 11: LEFT_ANKLE_LIFT
    {0.0f, 180.0f},   // 12: RIGHT_LEG_ROTATE
    {0.0f, 180.0f}    // 13: RIGHT_ANKLE_LIFT
}};

// Helper to get the calibrated home position for a servo
inline float get_home_pos(ServoChannel channel) {
    size_t index = static_cast<size_t>(channel);
    if (index < trims.size()) {
        return 90.0f + trims[index];
    }
    return 90.0f;
}

} // namespace ServoCalibration
