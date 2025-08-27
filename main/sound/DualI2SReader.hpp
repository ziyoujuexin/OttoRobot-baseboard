#pragma once

#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 定义麦克风数量和配置
#define NUM_MICS 4
#define I2S_SAMPLE_RATE 16000
#define I2S_DMA_BUFFER_SAMPLES 320 // for 20ms at 16kHz

class DualI2SReader {
public:
    DualI2SReader();
    ~DualI2SReader();

    /**
     * @brief 初始化双I2S控制器 (Master & Slave)
     * @return esp_err_t 成功返回ESP_OK
     */
    esp_err_t begin();

    /**
     * @brief 读取解交错后的多通道麦克风数据
     * @param mic_buffers 一个指针数组，指向您为每个麦克风数据分配的缓冲区
     * @param timeout 等待数据的超时时间
     * @return size_t 成功读取的采样点数，0表示超时或错误
     */
    size_t read(int32_t** mic_buffers, TickType_t timeout);

private:
    // I2S句柄
    i2s_chan_handle_t i2s0_rx_handle;
    i2s_chan_handle_t i2s1_rx_handle;

    // FreeRTOS 资源
    TaskHandle_t read_task_handle;
    QueueHandle_t data_queue;

    // 任务入口函数
    static void read_task_entry(void* arg);
    void read_task_impl();
};