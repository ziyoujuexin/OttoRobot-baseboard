#ifndef SERVO_HPP
#define SERVO_HPP

#include <stdint.h>

class Servo {
public:
    Servo();
    void init();
    void set_angle(uint8_t channel, uint8_t angle);
    void home_all();
};

#endif // SERVO_HPP