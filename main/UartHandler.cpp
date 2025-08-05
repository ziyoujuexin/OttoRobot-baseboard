#include "UartHandler.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "soc/gpio_num.h"

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
    if (len != 7) {
        ESP_LOGE(TAG, "[Line %d] Frame length error: %d", __LINE__, len);
        return false;
    }
    uint16_t header = (frame[0] << 8) | frame[1];
    if (header != FRAME_HEADER) {
        ESP_LOGE(TAG, "[Line %d] Frame header error: 0x%04X", __LINE__, header);
        return false;
    }
    if (frame[2] != SENDER_ID) {
        ESP_LOGE(TAG, "[Line %d] SENDER_ID error: 0x%02X", __LINE__, frame[2]);
        return false;
    }
    if (frame[3] != DATA_TYPE_MOTION) {
        ESP_LOGE(TAG, "[Line %d] DATA_TYPE_MOTION error: 0x%02X", __LINE__, frame[3]);
        return false;
    }
    if (frame[6] != FRAME_TAIL) {
        ESP_LOGE(TAG, "[Line %d] FRAME_TAIL error: 0x%02X", __LINE__, frame[6]);
        return false;
    }
    uint8_t checksum = frame[0] + frame[1] + frame[2] + frame[3] + frame[4];
    if ((checksum & 0xFF) != frame[5]) {
        ESP_LOGE(TAG, "[Line %d] Checksum error: calc=0x%02X, recv=0x%02X", __LINE__, (checksum & 0xFF), frame[5]);
        return false;
    }
    return true;
}

void UartHandler::receive_task_handler() {
    uint8_t data[UART_BUFFER_SIZE];
    uint8_t frame_buffer[7];
    int frame_index = 0;
    bool frame_started = false;

    ESP_LOGI(TAG, "UART receive task handler started.");

    while (1) {
        // 从UART读取数据
        int len = uart_read_bytes(UART_NUM, data, UART_BUFFER_SIZE, pdMS_TO_TICKS(50));
        
        if (len > 0) {
            // 遍历所有接收到的字节
            for (int i = 0; i < len; i++) {
                if (!frame_started) {
                    ESP_LOGI(TAG, "no frame_started");
                    // 寻找帧头 0xAA 0x55
                    if (frame_index == 0 && data[i] == (FRAME_HEADER >> 8)) {
                        frame_buffer[frame_index++] = data[i];
                    } else if (frame_index == 1 && data[i] == (uint8_t)FRAME_HEADER) {
                        frame_buffer[frame_index++] = data[i];
                        frame_started = true;
                    } else {
                        // 如果不是合法的帧头，则重置
                        frame_index = 0;
                    }
                } else {
                    ESP_LOGI(TAG, "frame_started");
                    // 已经找到帧头，开始接收帧的剩余部分
                    frame_buffer[frame_index++] = data[i];
                    
                    // 判断是否已接收完整的一帧（7字节）
                    if (frame_index >= 7) {
                        if (validate_frame(frame_buffer, 7)) { // 验证帧的有效性
                            motion_command_t cmd;
                            cmd.motion_type = frame_buffer[4]; // 提取运动类型
                            
                            // 关键点：调用 MotionController 的方法，而不是向全局队列发送
                            if (!m_motion_controller.queue_command(cmd)) {
                                ESP_LOGW(TAG, "Failed to queue motion command.");
                            } else {
                                ESP_LOGI(TAG, "Motion command %d queued.", cmd.motion_type);
                            }
                        } else {
                            ESP_LOGW(TAG, "Invalid frame received");
                        }
                        
                        // 处理完一帧后，重置状态，准备接收下一帧
                        frame_index = 0;
                        frame_started = false;
                    }
                }
            }
        }
    }
}