#pragma once

#include "driver/uart.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "esp32c3_szp.h"  // 包含esp32c3_szp.h来获取motion_command_t定义

// 移除重复的motion_command_t定义，使用esp32c3_szp.h中的定义

extern QueueHandle_t motion_queue;

void hardwire_init(void);
void motion_control_task(void *pvParameters);
void app_start(void);