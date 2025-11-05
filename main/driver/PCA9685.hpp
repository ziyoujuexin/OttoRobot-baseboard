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
    virtual void set_angle(uint8_t channel, float angle);
    virtual void home_all();

private:
    i2c_dev_t dev;
    uint16_t map_angle_to_pwm(float angle, float min_angle, float max_angle, uint16_t min_pulse_us, uint16_t max_pulse_us);
};

#endif // PCA9685_HPP
