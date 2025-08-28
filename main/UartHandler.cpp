#include "UartHandler.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "soc/gpio_num.h"
#include <vector>
#include <cstring> // For memcpy
#include "MotionController.hpp" // For FaceLocation and motion commands

#define UART_NUM           UART_NUM_1
#define UART_TX_PIN        GPIO_NUM_47
#define UART_RX_PIN        GPIO_NUM_48
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

    esp_timer_create_args_t timer_args = {
        .callback = &wake_word_timer_callback,
        .arg = this,
        .name = "wake_word_timer"
    };
    esp_timer_create(&timer_args, &m_wake_word_timer);

    xTaskCreate(start_task_wrapper, "uart_receive_task", 4096, this, 5, NULL);
    ESP_LOGI(TAG, "UART Handler initialized and task started.");
}

void UartHandler::wake_word_timer_callback(void* arg) {
    UartHandler* handler = static_cast<UartHandler*>(arg);
    handler->m_isWakeWordDetected = false;
    ESP_LOGI(TAG, "Wake word timer expired, m_isWakeWordDetected set to false.");
}

void UartHandler::start_wake_word_timer() {
    const uint8_t timeout_seconds = 4;
    esp_timer_start_once(m_wake_word_timer, timeout_seconds * 1000000);
    ESP_LOGI(TAG, "Started %d second timer for wake word detection.", timeout_seconds);
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
                            motion_command_t cmd;
                            uint8_t motion_type = frame_buffer[5]; // Payload[0] is the command
                            cmd.motion_type = motion_type;

                            // Check if the command is for face tracking
                            if (motion_type == MOTION_FACE_TRACE) {
                                const size_t expected_data_len = 10; // x(2), y(2), w(2), h(2), detected(1), repeat(1)
                                const size_t actual_data_len = payload_len > 0 ? payload_len - 1 : 0;

                                if (actual_data_len == expected_data_len) {
                                    FaceLocation fl;
                                    const uint8_t* data_ptr = frame_buffer.data() + 6; // Data starts after the command byte

                                    // Manually deserialize from little-endian byte stream
                                    fl.x = data_ptr[0] | (data_ptr[1] << 8);
                                    fl.y = data_ptr[2] | (data_ptr[3] << 8);
                                    fl.w = data_ptr[4] | (data_ptr[5] << 8);
                                    fl.h = data_ptr[6] | (data_ptr[7] << 8);
                                    fl.detected = (data_ptr[8] != 0);

                                    // Queue FaceLocation directly to the new queue
                                    if (!m_motion_controller.queue_face_location(fl)) {
                                        ESP_LOGW(TAG, "Failed to queue face location.");
                                    } else {
                                        ESP_LOGD(TAG, "FaceLocation queued: x=%d, y=%d, w=%d, h=%d, detected=%d", fl.x, fl.y, fl.w, fl.h, fl.detected);
                                    }

                                } else {
                                    ESP_LOGW(TAG, "Invalid payload length for face trace. Expected %d, got %d.", 
                                             expected_data_len, actual_data_len);
                                    // Clear buffer and continue to next frame to avoid processing a bad command
                                    frame_buffer.clear();
                                    frame_started = false;
                                    continue; 
                                }
                            } else if(motion_type == MOTION_WAKE_DETECT) {
                                ESP_LOGI(TAG, "Wake word detected command received.");
                                m_isWakeWordDetected = true;
                                start_wake_word_timer();
                                // No additional params expected for wake detect
                                // Queue the command for the motion controller
                                if (!m_motion_controller.queue_command(cmd)) {
                                    ESP_LOGW(TAG, "Failed to queue motion command.");
                                } else {
                                    ESP_LOGD(TAG, "Command %d with %d bytes of params queued.", cmd.motion_type, cmd.params.size());
                                }
                            } else {
                                // Generic handling for other commands
                                if (payload_len > 1) {
                                    cmd.params.assign(frame_buffer.begin() + 6, frame_buffer.begin() + 6 + (payload_len - 1));
                                }
                                // Queue the command for the motion controller
                                if (!m_motion_controller.queue_command(cmd)) {
                                    ESP_LOGW(TAG, "Failed to queue motion command.");
                                } else {
                                    ESP_LOGD(TAG, "Command %d with %d bytes of params queued.", cmd.motion_type, cmd.params.size());
                                }
                            }
                        } else {
                            ESP_LOGW(TAG, "Invalid frame received");
                        }
                        
                        // --- 阶段3: 重置状态以寻找下一个帧 ---
                        frame_buffer.clear();
                        frame_started = false;
                    }
                }
            }
        }
    }
}

