#ifndef PCA9685_HPP
#define PCA9685_HPP

#include "servo.hpp"
extern "C" {
#include <pca9685.h>
}

// Default I2C configuration, you may need to change these based on your hardware.
#define PCA9685_I2C_ADDR PCA9685_ADDR_BASE 
#define I2C_PORT 0
#define SDA_PIN 23
#define SCL_PIN 22
#define PWM_FREQ_HZ             60      // 舵机PWM频率

class PCA9685 : public Servo {
public:
    PCA9685();
    ~PCA9685() override = default;

    void init() override;
    void set_angle(uint8_t channel, uint16_t angle) override;
    void home_all() override;

private:
    i2c_dev_t dev;
    uint16_t map_angle_to_pwm(uint16_t angle);
};

#endif // PCA9685_HPP
