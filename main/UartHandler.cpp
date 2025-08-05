#include "UartHandler.hpp"
#include "driver/uart.h"
#include "esp_log.h"
#include "soc/gpio_num.h"

#define UART_NUM           UART_NUM_1
#define UART_TX_PIN        GPIO_NUM_11
#define UART_RX_PIN        GPIO_NUM_10
#define UART_BAUD_RATE     115200
#define UART_BUFFER_SIZE   256
#define FRAME_HEADER       0x55AA
#define FRAME_TAIL         0xFF
#define SENDER_ID          0x01
#define DATA_TYPE_MOTION   0x10

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
    // ... 校验逻辑和 esp32c3_szp.c 中的 validate_frame 一样 ...
    if (len != 7) return false;
    uint16_t header = (frame[0] << 8) | frame[1];
    if (header != FRAME_HEADER) return false;
    if (frame[2] != SENDER_ID || frame[3] != DATA_TYPE_MOTION) return false;
    if (frame[6] != FRAME_TAIL) return false;
    uint8_t checksum = frame[0] + frame[1] + frame[2] + frame[3] + frame[4];
    if ((checksum & 0xFF) != frame[5]) return false;
    return true;
}

void UartHandler::receive_task_handler() {
    // ... 接收和组帧逻辑和 esp32c3_szp.c 中的 uart_receive_task 一样 ...
    uint8_t data[UART_BUFFER_SIZE];
    uint8_t frame_buffer[7];
    int frame_index = 0;
    bool frame_started = false;

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, UART_BUFFER_SIZE, pdMS_TO_TICKS(20));
        if (len > 0) {
            if (validate_frame(frame_buffer, 7)) {
                motion_command_t cmd;
                cmd.motion_type = frame_buffer[4];
                
                // 关键点：调用 MotionController 的方法，而不是向全局队列发送
                if (!m_motion_controller.queue_command(cmd)) {
                    ESP_LOGW(TAG, "Failed to queue motion command.");
                } else {
                    ESP_LOGI(TAG, "Motion command %d queued.", cmd.motion_type);
                }
            }
            // ... 重置帧接收状态 ...
        }
    }
}