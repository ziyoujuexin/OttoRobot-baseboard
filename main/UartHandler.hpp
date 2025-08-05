#pragma once

#include "MotionController.hpp"
#include <memory>

class UartHandler {
public:
    explicit UartHandler(MotionController& motion_controller);
    void init();

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
};