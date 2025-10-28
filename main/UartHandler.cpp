#include "UartHandler.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "soc/gpio_num.h"
#include <vector>
#include <string> // Required for std::string and std::stof
#include <cstring> // For memcpy

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

UartHandler::UartHandler(MotionController& controller, AnimationPlayer* anim_player, FaceLocationCallback callback)
    : m_motion_controller(controller), m_anim_player(anim_player), m_face_location_callback(callback) {}

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
    if (len < 7) return false;
    uint16_t header = (frame[0] << 8) | frame[1];
    if (header != FRAME_HEADER) return false;
    if (frame[2] != SENDER_ID) return false;
    if (frame[3] != DATA_TYPE_MOTION) return false;
    uint8_t payload_len = frame[4];
    if (len != 7 + payload_len) return false;
    if (frame[len - 1] != FRAME_TAIL) return false;
    uint8_t checksum = 0;
    for (size_t i = 0; i < len - 2; ++i) {
        checksum += frame[i];
    }
    if ((checksum & 0xFF) != frame[len - 2]) return false;
    return true;
}

void UartHandler::receive_task_handler() {
    uint8_t data[UART_BUFFER_SIZE];
    std::vector<uint8_t> binary_buffer;
    std::vector<char> text_buffer;
    bool in_binary_frame = false;

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, UART_BUFFER_SIZE, pdMS_TO_TICKS(50));
        if (len <= 0) continue;

        for (int i = 0; i < len; i++) {
            uint8_t byte = data[i];

            if (in_binary_frame) {
                binary_buffer.push_back(byte);
                if (binary_buffer.size() >= 5) { // Have enough to check payload length
                    uint8_t payload_len = binary_buffer[4];
                    size_t total_frame_len = 7 + payload_len;
                    if (binary_buffer.size() >= total_frame_len) {
                        if (validate_frame(binary_buffer.data(), total_frame_len)) {
                            uint8_t motion_type = binary_buffer[5];
                            if (motion_type == MOTION_FACE_TRACE) {
                                const size_t expected_data_len = 10;
                                const size_t actual_data_len = payload_len > 0 ? payload_len - 1 : 0;
                                if (actual_data_len == expected_data_len) {
                                    FaceLocation fl;
                                    const uint8_t* data_ptr = binary_buffer.data() + 6;
                                    fl.x = data_ptr[0] | (data_ptr[1] << 8);
                                    fl.y = data_ptr[2] | (data_ptr[3] << 8);
                                    fl.w = data_ptr[4] | (data_ptr[5] << 8);
                                    fl.h = data_ptr[6] | (data_ptr[7] << 8);
                                    fl.detected = (data_ptr[8] != 0);
                                    if (m_face_location_callback) m_face_location_callback(fl);
                                }
                            } else if (motion_type == MOTION_WAKE_DETECT) {
                                ESP_LOGI(TAG, "Wake word detected.");
                                m_isWakeWordDetected = true;
                                start_wake_word_timer();
                            } else if (motion_type == MOTION_PLAY_ANIMATION) {
                                if (m_anim_player) {
                                    const char* anim_name = (const char*)(binary_buffer.data() + 6);
                                    uint8_t anim_name_len = payload_len > 0 ? payload_len - 1 : 0;
                                    std::string anim_name_str(anim_name, anim_name_len);
                                    ESP_LOGI(TAG, "Queueing one-shot animation: %s", anim_name_str.c_str());
                                    m_anim_player->playOneShotAnimation(anim_name_str);
                                }
                            } else if (motion_type == MOTION_FACE_END) {
                                ESP_LOGI(TAG, "Face end detected, stopping all motions.");
                                m_motion_controller.queue_command({MOTION_STOP, {}});
                            }
                        }
                        binary_buffer.clear();
                        in_binary_frame = false;
                    }
                }
            } else { // Not in a binary frame, so treat as potential text or start of binary
                if (binary_buffer.empty() && byte == (FRAME_HEADER >> 8)) {
                    binary_buffer.push_back(byte);
                } else if (binary_buffer.size() == 1 && byte == (uint8_t)FRAME_HEADER) {
                    binary_buffer.push_back(byte);
                    in_binary_frame = true;
                    text_buffer.clear(); // Clear any partial text command
                } else {
                    binary_buffer.clear(); // Not a valid binary start, reset
                    // Handle as text command
                    if (byte == '\n' || byte == '\r') {
                        if (!text_buffer.empty()) {
                            text_buffer.push_back('\0');
                            std::string line(text_buffer.data());
                            ESP_LOGI(TAG, "Received text command: %s", line.c_str());
                            if (line.rfind("set_filter_alpha ", 0) == 0) {
                                try {
                                    float alpha = std::stof(line.substr(17));
                                    m_motion_controller.set_filter_alpha(alpha);
                                } catch (const std::exception& e) {
                                    ESP_LOGE(TAG, "Invalid argument for set_filter_alpha: %s", e.what());
                                }
                            }
                            text_buffer.clear();
                        }
                    } else if (isprint(byte)) {
                        if (text_buffer.size() < 100) { // Prevent buffer overflow
                           text_buffer.push_back(byte);
                        }
                    }
                }
            }
        }
    }
}
