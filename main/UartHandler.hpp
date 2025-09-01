#pragma once

#include <functional>
#include "motion_manager/Motion_types.hpp"
#include "esp_timer.h"

class UartHandler {
public:
    using FaceLocationCallback = std::function<void(const FaceLocation&)>;

    explicit UartHandler(FaceLocationCallback callback);
    void init();

    bool m_isWakeWordDetected = false;

private:
    FaceLocationCallback m_face_location_callback;

    void receive_task_handler();
    bool validate_frame(uint8_t *frame, int len);

    static void start_task_wrapper(void* _this) {
        static_cast<UartHandler*>(_this)->receive_task_handler();
    }

    esp_timer_handle_t m_wake_word_timer;
    static void wake_word_timer_callback(void* arg);
    void start_wake_word_timer();
};