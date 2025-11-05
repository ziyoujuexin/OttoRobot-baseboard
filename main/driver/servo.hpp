#ifndef SERVO_HPP
#define SERVO_HPP

#include <stdint.h>

class Servo {
public:
    Servo() = default;
    virtual ~Servo() = default;

    virtual void init() = 0;
    virtual void set_angle(uint8_t channel, float angle) = 0;
    virtual void home_all() = 0;
};

#endif // SERVO_HPP