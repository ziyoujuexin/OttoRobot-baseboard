#ifndef PCA9685_HPP
#define PCA9685_HPP

#include "servo.hpp"
extern "C" {
#include <pca9685.h>
}

// Default I2C configuration, you may need to change these based on your hardware.
#define PCA9685_I2C_ADDR PCA9685_ADDR_BASE 
#define I2C_PORT 0
#define SDA_PIN 0
#define SCL_PIN 1
#define PWM_FREQ_HZ             60      // 舵机PWM频率

#define SERVO_MIN_PULSE        150     // 0度对应的脉冲宽度计数值
#define SERVO_MAX_PULSE        600     // 180度对应的脉冲宽度计数值

// 舵机安全限制 (可选，但建议保留)
#define SERVO_ABSOLUTE_MIN_PULSE  100  // 绝对最小脉冲宽度计数值
#define SERVO_ABSOLUTE_MAX_PULSE  700  // 绝对最大脉冲宽度计数值

class PCA9685 : public Servo {
public:
    PCA9685();
    ~PCA9685() override = default;

    void init() override;
    void set_angle(uint8_t channel, uint8_t angle) override;
    void home_all() override;

private:
    i2c_dev_t dev;
    uint16_t map_angle_to_pwm(uint8_t angle);
};

#endif // PCA9685_HPP
