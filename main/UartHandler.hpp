#pragma once

#include "MotionController.hpp"
#include <memory>
#include "esp_timer.h"

class UartHandler {
public:
    explicit UartHandler(MotionController& motion_controller);
    void init();

    bool m_isWakeWordDetected = false; // Public member to track wake word detection

private:
    // 构造函数私有化
    UartHandler(std::shared_ptr<MotionController> motion_controller);

    // 成员变量现在是一个 shared_ptr
    MotionController& m_motion_controller;

    void receive_task_handler();
    bool validate_frame(uint8_t *frame, int len);

    static void start_task_wrapper(void* _this) {
        static_cast<UartHandler*>(_this)->receive_task_handler();
    }

    esp_timer_handle_t m_wake_word_timer;
    static void wake_word_timer_callback(void* arg);
    void start_wake_word_timer();
};