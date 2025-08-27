#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_vad.h"
#include <functional>

class VAD {
public:
    VAD(int sample_rate, int frame_length_ms);
    ~VAD();

    void feed(const int16_t** audio_buffers, int num_samples, int channel);
    void on_vad_state_change(std::function<void(bool speaking)> callback);

private:
    static void vad_task(void* arg);

    vad_handle_t vad_inst_;
    TaskHandle_t task_handle_;
    QueueHandle_t data_queue_;
    
    int sample_rate_;
    bool is_speaking_;
    int frame_length_ms_;
    int frame_size_samples_; // 每个音频帧的样本数
    std::function<void(bool speaking)> vad_state_change_callback_;
};