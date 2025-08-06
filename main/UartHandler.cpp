#include "UartHandler.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "soc/gpio_num.h"
#include <vector>

#define UART_NUM           UART_NUM_1
#define UART_TX_PIN        GPIO_NUM_18
#define UART_RX_PIN        GPIO_NUM_19
#define UART_BAUD_RATE     115200
#define UART_BUFFER_SIZE   256
#define FRAME_HEADER       0x55AA
#define FRAME_TAIL         0xBB
#define SENDER_ID          0x01
#define DATA_TYPE_MOTION   0x02

static const char* TAG = "UartHandler";

UartHandler::UartHandler(MotionController& motion_controller) : m_motion_controller(motion_controller) {}

void UartHandler::init() {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 0, NULL, 0);

    xTaskCreate(start_task_wrapper, "uart_receive_task", 4096, this, 5, NULL);
    ESP_LOGI(TAG, "UART Handler initialized and task started.");
}

bool UartHandler::validate_frame(uint8_t *frame, int len) {
    // 最小帧长度：帧头(2) + ID(1) + 类型(1) + 长度(1) + 校验(1) + 帧尾(1) = 7
    if (len < 7) {
        ESP_LOGE(TAG, "[Line %d] Frame length too short: %d", __LINE__, len);
        return false;
    }
    // 验证帧头
    uint16_t header = (frame[0] << 8) | frame[1];
    if (header != FRAME_HEADER) {
        ESP_LOGE(TAG, "[Line %d] Frame header error: 0x%04X", __LINE__, header);
        return false;
    }
    // 验证发送者ID
    if (frame[2] != SENDER_ID) {
        ESP_LOGE(TAG, "[Line %d] SENDER_ID error: 0x%02X", __LINE__, frame[2]);
        return false;
    }
    // 验证数据类型（仅支持DATA_TYPE_MOTION）
    if (frame[3] != DATA_TYPE_MOTION) {
        ESP_LOGE(TAG, "[Line %d] DATA_TYPE_MOTION error: 0x%02X", __LINE__, frame[3]);
        return false;
    }
    // 验证帧长度与长度字段一致
    uint8_t payload_len = frame[4];
    if (len != 7 + payload_len) {
        ESP_LOGE(TAG, "[Line %d] Frame length mismatch: expected %d, got %d", __LINE__, 7 + payload_len, len);
        return false;
    }
    // 验证帧尾
    if (frame[len - 1] != FRAME_TAIL) {
        ESP_LOGE(TAG, "[Line %d] FRAME_TAIL error: 0x%02X", __LINE__, frame[len - 1]);
        return false;
    }
    // 计算校验和（从帧头到负载结束）
    uint8_t checksum = 0;
    for (size_t i = 0; i < len - 2; ++i) {
        checksum += frame[i];
    }
    if ((checksum & 0xFF) != frame[len - 2]) {
        ESP_LOGE(TAG, "[Line %d] Checksum error: calc=0x%02X, recv=0x%02X", __LINE__, (checksum & 0xFF), frame[len - 2]);
        return false;
    }
    return true;
}

void UartHandler::receive_task_handler() {
    uint8_t data[UART_BUFFER_SIZE];
    std::vector<uint8_t> frame_buffer;
    bool frame_started = false;

    ESP_LOGI(TAG, "UART receive task handler started.");

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, UART_BUFFER_SIZE, pdMS_TO_TICKS(50));

        if (len <= 0) {
            continue;
        }

        for (int i = 0; i < len; i++) {
            // --- 阶段1: 寻找并缓存一个完整的帧 ---
            if (!frame_started) {
                // 寻找帧头 0x55
                if (frame_buffer.empty() && data[i] == (FRAME_HEADER >> 8)) {
                    frame_buffer.push_back(data[i]);
                }
                // 寻找帧头 0xAA
                else if (frame_buffer.size() == 1 && data[i] == (uint8_t)FRAME_HEADER) {
                    frame_buffer.push_back(data[i]);
                    frame_started = true;
                } else {
                    frame_buffer.clear(); // 不是有效的帧头，清空缓存
                }
            } else {
                // 已经找到帧头，开始接收帧的剩余部分
                frame_buffer.push_back(data[i]);

                // 检查是否已接收到包含负载长度的头部 (头2+ID1+类型1+长度1 = 5字节)
                if (frame_buffer.size() >= 5) {
                    uint8_t payload_len = frame_buffer[4];
                    size_t total_frame_len = 7 + payload_len; // 7 = 协议固定开销

                    // 检查是否已接收到一个完整的帧
                    if (frame_buffer.size() >= total_frame_len) {
                        
                        // --- 阶段2: 验证并解析帧 ---
                        if (validate_frame(frame_buffer.data(), total_frame_len)) {
                            // 帧有效，解析Payload
                            motion_command_t cmd = {0}; // 初始化结构体

                            // Payload[0] (帧内索引5) 是具体指令
                            cmd.motion_type = frame_buffer[5];

                            // 如果负载长度大于1，则包含参数
                            if (payload_len > 1) {
                                // Payload[1] (帧内索引6) 是参数
                                cmd.param = frame_buffer[6]; 
                            }

                            // 将解析出的完整指令放入队列
                            if (!m_motion_controller.queue_command(cmd)) {
                                ESP_LOGW(TAG, "Failed to queue motion command.");
                            } else {
                                if (payload_len > 1) {
                                    ESP_LOGI(TAG, "Command %d with param %d queued.", cmd.motion_type, cmd.param);
                                } else {
                                    ESP_LOGI(TAG, "Command %d queued.", cmd.motion_type);
                                }
                            }
                        } else {
                            ESP_LOGW(TAG, "Invalid frame received");
                        }
                        
                        // --- 阶段3: 重置状态以寻找下一个帧 ---
                        // 注意：这里可以优化为只移除已处理的帧，而不是清空整个缓冲区
                        frame_buffer.clear();
                        frame_started = false;
                    }
                }
            }
        }
    }
}