#include "VAD.hpp"
#include "esp_log.h"

static const char* TAG = "VAD";

// 构造函数：初始化VAD实例和任务
VAD::VAD(int sample_rate, int frame_length_ms)
    : vad_inst_(nullptr),
      task_handle_(nullptr),
      data_queue_(nullptr),
      sample_rate_(sample_rate),
      is_speaking_(false),
      frame_length_ms_(frame_length_ms) {

    // 1. 创建VAD实例，模式0最敏感，模式4最不敏感
    vad_inst_ = vad_create(VAD_MODE_0);
    if (!vad_inst_) {
        ESP_LOGE(TAG, "Failed to create VAD instance");
        return;
    }
    
    // 计算每帧的样本数
    frame_size_samples_ = (sample_rate_ * frame_length_ms_) / 1000;

    // 2. 创建数据队列
    // 队列项大小为一个音频帧的大小
    data_queue_ = xQueueCreate(10, frame_size_samples_ * sizeof(int16_t));
    if (!data_queue_) {
        ESP_LOGE(TAG, "Failed to create data queue");
        vad_destroy(vad_inst_);
        return;
    }

    // 3. 创建处理任务
    xTaskCreatePinnedToCore(this->vad_task, "VAD_Task", 4096, this, 5, &task_handle_, 1);
}

// 析构函数：释放资源
VAD::~VAD() {
    if (task_handle_) vTaskDelete(task_handle_);
    if (data_queue_) vQueueDelete(data_queue_);
    if (vad_inst_) vad_destroy(vad_inst_);
}

// 喂送16位单声道音频数据
void VAD::feed(const int16_t** audio_buffers, int num_samples, int channel) {
    if (!data_queue_ || !vad_inst_) return;

    // 检查传入的样本数是否与VAD期望的帧大小一致
    if (num_samples != this->frame_size_samples_) {
        ESP_LOGE(TAG, "Input sample count %d does not match VAD frame size %d", num_samples, this->frame_size_samples_);
        return;
    }

    // 将转换后的16位单声道数据帧发送到队列
    if (xQueueSend(data_queue_, audio_buffers[channel], pdMS_TO_TICKS(100)) != pdTRUE) {
        // 如果需要，可以在这里处理队列满的情况
    }
}

// 注册状态变化回调函数
void VAD::on_vad_state_change(std::function<void(bool speaking)> callback) {
    vad_state_change_callback_ = callback;
}

// VAD处理任务
void VAD::vad_task(void* arg) {
    VAD* self = static_cast<VAD*>(arg);
    int16_t* frame_buffer = (int16_t*)malloc(self->frame_size_samples_ * sizeof(int16_t));

    if (!frame_buffer) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer!");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "VAD task started.");

    while (true) {
        // 从队列接收一帧音频数据
        if (xQueueReceive(self->data_queue_, frame_buffer, portMAX_DELAY) == pdTRUE) {
            
            // 调用esp_vad核心处理函数
            vad_state_t vad_state = vad_process(self->vad_inst_, frame_buffer, self->sample_rate_, self->frame_length_ms_);

            if (vad_state == VAD_SPEECH) {
                if (!self->is_speaking_) {
                    self->is_speaking_ = true;
                    if (self->vad_state_change_callback_) {
                        self->vad_state_change_callback_(true);
                    }
                }
            } else {
                if (self->is_speaking_) {
                    self->is_speaking_ = false;
                    if (self->vad_state_change_callback_) {
                        self->vad_state_change_callback_(false);
                    }
                }
            }
        }
    }

    free(frame_buffer);
    vTaskDelete(NULL);
}
