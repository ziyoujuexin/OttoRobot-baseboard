#ifndef UART_HPP
#define UART_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// Function to set the motion queue handle
void uart_set_motion_queue(QueueHandle_t queue);

// Public function declarations
void uart_init(void);
void uart_receive_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // UART_HPP
