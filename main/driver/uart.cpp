#include "uart.hpp"
#include "esp_log.h"
#include "string.h"
#include "driver/uart.h"
#include "config.h"
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

static const char *UART_TAG = "UART";
// static QueueHandle_t s_motion_queue = NULL;
// 向前声明
extern QueueHandle_t s_motion_queue;

void uart_set_motion_queue(QueueHandle_t queue) {
    s_motion_queue = queue;
}

void uart_init(void) {
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
    ESP_LOGI(UART_TAG, "UART initialized on IO%d (TX) and IO%d (RX)", UART_TX_PIN, UART_RX_PIN);
}

static bool validate_frame(uint8_t *frame, int len) {
    if (len != 7) return false;
    if (((frame[0] << 8) | frame[1]) != FRAME_HEADER) return false;
    if (frame[2] != SENDER_ID || frame[3] != DATA_TYPE_MOTION) return false;
    if (frame[6] != FRAME_TAIL) return false;
    uint8_t checksum = frame[0] + frame[1] + frame[2] + frame[3] + frame[4];
    if ((checksum & 0xFF) != frame[5]) return false;
    return true;
}

void uart_receive_task(void *pvParameters) {
    uint8_t data[UART_BUFFER_SIZE];
    uint8_t frame_buffer[7];
    int frame_index = 0;
    bool frame_started = false;

    ESP_LOGI(UART_TAG, "UART receive task started");

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, UART_BUFFER_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                if (!frame_started) {
                    if (frame_index == 0 && data[i] == 0xAA) {
                        frame_buffer[frame_index++] = data[i];
                    } else if (frame_index == 1 && data[i] == 0x55) {
                        frame_buffer[frame_index++] = data[i];
                        frame_started = true;
                    } else {
                        frame_index = 0;
                    }
                } else {
                    frame_buffer[frame_index++] = data[i];
                    if (frame_index >= 7) {
                        if (validate_frame(frame_buffer, 7)) {
                            motion_command_t cmd;
                            cmd.motion_type = frame_buffer[4];
                            if (s_motion_queue != NULL && xQueueSend(s_motion_queue, &cmd, 0) != pdTRUE) {
                                ESP_LOGW(UART_TAG, "Motion queue full, dropping command");
                            }
                        } else {
                            ESP_LOGW(UART_TAG, "Invalid frame received");
                        }
                        frame_index = 0;
                        frame_started = false;
                    }
                }
            }
        }
    }
}