#include "DualI2SReader.hpp"
#include "esp_log.h"

static const char *TAG = "DualI2SReader";

// ------ I2S 引脚定义 ------
// #define I2S_BCLK_PIN    GPIO_NUM_5
// #define I2S_WS_PIN      GPIO_NUM_4
// #define I2S0_DIN_PIN    GPIO_NUM_6  // Mic 0, 1
// #define I2S1_DIN_PIN    GPIO_NUM_7  // Mic 2, 3

// suitable for P4
#define I2S_BCLK_PIN    GPIO_NUM_30
#define I2S_WS_PIN      GPIO_NUM_31
#define I2S0_DIN_PIN    GPIO_NUM_29  // Mic 0, 1
#define I2S1_DIN_PIN    GPIO_NUM_28  // Mic 2, 3

DualI2SReader::DualI2SReader() : i2s0_rx_handle(nullptr), i2s1_rx_handle(nullptr), read_task_handle(nullptr), data_queue(nullptr) {}

DualI2SReader::~DualI2SReader() {
    
    if (read_task_handle) {
        vTaskDelete(read_task_handle);
    }
    if (data_queue) {
        vQueueDelete(data_queue);
    }
    if (i2s0_rx_handle) {
        i2s_channel_disable(i2s0_rx_handle);
        i2s_del_channel(i2s0_rx_handle);
    }
    if (i2s1_rx_handle) {
        i2s_channel_disable(i2s1_rx_handle);
        i2s_del_channel(i2s1_rx_handle);
    }
}

esp_err_t DualI2SReader::begin() {
    ESP_LOGI(TAG, "Initializing dual I2S controllers...");

    // 1. 创建一个对主从都有效的标准配置
    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = I2S_SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_384,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {(gpio_num_t)0}, 
    };

    // 2. 初始化 I2S0 (主模式)
    i2s_chan_config_t i2s0_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s0_chan_cfg.dma_desc_num = 4;
    i2s0_chan_cfg.dma_frame_num = I2S_DMA_BUFFER_SAMPLES;
    ESP_ERROR_CHECK(i2s_new_channel(&i2s0_chan_cfg, NULL, &i2s0_rx_handle));
    
    std_cfg.gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = I2S_BCLK_PIN,
        .ws = I2S_WS_PIN,
        .dout = I2S_GPIO_UNUSED,
        .din = I2S0_DIN_PIN
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s0_rx_handle, &std_cfg));

    // 3. 初始化 I2S1 (从模式)
    i2s_chan_config_t i2s1_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
    i2s1_chan_cfg.dma_desc_num = 4;
    i2s1_chan_cfg.dma_frame_num = I2S_DMA_BUFFER_SAMPLES;
    ESP_ERROR_CHECK(i2s_new_channel(&i2s1_chan_cfg, NULL, &i2s1_rx_handle));

    std_cfg.gpio_cfg.din = I2S1_DIN_PIN; 
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s1_rx_handle, &std_cfg));

    // 4. 创建数据队列和读取任务
    data_queue = xQueueCreate(1, sizeof(int32_t**));
    if (!data_queue) {
        ESP_LOGE(TAG, "Failed to create data queue");
        return ESP_FAIL;
    }

    BaseType_t result = xTaskCreatePinnedToCore(read_task_entry, "I2SReadTask", 4096, this, 5, &read_task_handle, 1);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create I2S read task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

size_t DualI2SReader::read(int32_t** mic_buffers, TickType_t timeout) {
    int32_t** received_buffers;
    if (xQueueReceive(data_queue, &received_buffers, timeout) == pdTRUE) {
        for (int i = 0; i < NUM_MICS; i++) {
            memcpy(mic_buffers[i], received_buffers[i], I2S_DMA_BUFFER_SAMPLES * sizeof(int32_t));
        }
        return I2S_DMA_BUFFER_SAMPLES;
    }
    return 0;
}

void DualI2SReader::read_task_entry(void* arg) {
    static_cast<DualI2SReader*>(arg)->read_task_impl();
}

void DualI2SReader::read_task_impl() {
    // 任务内部分配缓冲区
    size_t dma_buf_size = I2S_DMA_BUFFER_SAMPLES * 2 * sizeof(int32_t); // 2 channels per controller
    int32_t* i2s0_buf = (int32_t*)malloc(dma_buf_size);
    int32_t* i2s1_buf = (int32_t*)malloc(dma_buf_size);

    int32_t** mic_data_ptr = new int32_t*[NUM_MICS];
    for (int i = 0; i < NUM_MICS; i++) {
        mic_data_ptr[i] = new int32_t[I2S_DMA_BUFFER_SAMPLES];
    }
    
    i2s_channel_enable(i2s0_rx_handle);
    i2s_channel_enable(i2s1_rx_handle);

    while (1) {
        size_t bytes_read0, bytes_read1;
        i2s_channel_read(i2s0_rx_handle, i2s0_buf, dma_buf_size, &bytes_read0, portMAX_DELAY);
        i2s_channel_read(i2s1_rx_handle, i2s1_buf, dma_buf_size, &bytes_read1, portMAX_DELAY);

        int samples_read = bytes_read0 / sizeof(int32_t) / 2;

        for (int i = 0; i < samples_read; i++) {
            mic_data_ptr[0][i] = i2s0_buf[i * 2 + 0];
            mic_data_ptr[1][i] = i2s0_buf[i * 2 + 1];
            mic_data_ptr[2][i] = i2s1_buf[i * 2 + 0];
            mic_data_ptr[3][i] = i2s1_buf[i * 2 + 1];
        }

        xQueueOverwrite(data_queue, &mic_data_ptr);
    }
}