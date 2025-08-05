#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_heap_caps.h"  // 添加内存检查头文件
#include <inttypes.h>        // 添加PRIu32宏定义
#include "esp32c3_szp.h"
#include "app_process.h"
#include "motion_control.h"

static const char *TAG = "MAIN";

// 定义全局队列变量
QueueHandle_t motion_queue = NULL;

void app_main(void)
{
    ESP_LOGI(TAG, "=== 程序开始执行 ===");
    ESP_LOGI(TAG, "ESP32-C3 机器人控制系统启动");
    
    // 确保FreeRTOS完全初始化
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // 添加详细的内存和系统状态检查
    // 第26行修复
    ESP_LOGI(TAG, "motion_command_t 结构体大小: %zu bytes", sizeof(motion_command_t));
    ESP_LOGI(TAG, "总可用堆内存: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "最大可分配块: %zu bytes", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    
    // 直接使用动态分配，避免静态分配的复杂性
    ESP_LOGI(TAG, "开始创建运动队列（动态分配）...");
    motion_queue = xQueueCreate(10, sizeof(motion_command_t));
    
    // 在队列创建成功后添加详细诊断
    if (motion_queue == NULL) {
        ESP_LOGE(TAG, "*** 队列创建失败，内存不足 ***");
        ESP_LOGI(TAG, "当前可用内存: %" PRIu32 " bytes", esp_get_free_heap_size());
        return;
    }
    
    // 添加详细的队列状态检查
    ESP_LOGI(TAG, "=== 队列创建诊断开始 ===");
    ESP_LOGI(TAG, "队列句柄地址: %p", (void*)motion_queue);
    // 第46行修复 - 改为以下两种方式之一：
    // 方式1：使用%p格式（推荐）
    ESP_LOGI(TAG, "队列句柄值: %p", (void*)motion_queue);
    // 方式2：或者使用PRIx32格式
    // ESP_LOGI(TAG, "队列句柄值: 0x%08" PRIx32, (uint32_t)(uintptr_t)motion_queue);
    // ...
    ESP_LOGI(TAG, "队列可用空间: %" PRIu32, (uint32_t)uxQueueSpacesAvailable(motion_queue));
    ESP_LOGI(TAG, "队列消息大小: %zu", sizeof(motion_command_t));
    ESP_LOGI(TAG, "当前堆内存: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "=== 队列创建诊断结束 ===");
    
    // **移除以下测试代码**
    // motion_command_t test_cmd = {MOTION_STOP};
    // BaseType_t result = xQueueSend(motion_queue, &test_cmd, 0);
    // ESP_LOGI(TAG, "队列测试发送结果: %s", (result == pdTRUE) ? "成功" : "失败");
    
    ESP_LOGI(TAG, "队列创建完成，开始硬件初始化");
    
    // 硬件初始化
    ESP_LOGI(TAG, "开始硬件初始化...");
    hardwire_init();
    ESP_LOGI(TAG, "硬件初始化完成");
    
    // 启动应用任务
    ESP_LOGI(TAG, "开始启动应用任务...");
    app_start();
    ESP_LOGI(TAG, "系统启动完成，等待S3串口指令");
    
    motion_command_t received_cmd;
    
    while (1) {
        // 添加心跳日志
        static int heartbeat_count = 0;
        if (++heartbeat_count % 50 == 0) {  // 每5秒输出一次心跳
            ESP_LOGI(TAG, "系统运行正常，等待指令中... (心跳: %d)", heartbeat_count);
        }
         // motion_walk(2, 500, 1);
        // 检查串口命令队列
        if (xQueueReceive(motion_queue, &received_cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGI(TAG, "*** 接收到S3指令: %d ***", received_cmd.motion_type);
            
            switch (received_cmd.motion_type) {
                case MOTION_FORWARD:
                    ESP_LOGI(TAG, ">>> 执行前进动作 <<<");
                    motion_walk(2, 500, 1);
                    ESP_LOGI(TAG, "前进动作完成");
                    break;
                    
                case MOTION_BACKWARD:
                    ESP_LOGI(TAG, "执行后退动作");
                    motion_walk(2, 500, -1);
                    break;
                    
                case MOTION_LEFT:
                    ESP_LOGI(TAG, "执行左转动作");
                    motion_turn(2, 500, 1);
                    break;
                    
                case MOTION_RIGHT:
                    ESP_LOGI(TAG, "执行右转动作");
                    motion_turn(2, 500, -1);
                    break;
                    
                case MOTION_STOP:
                    ESP_LOGI(TAG, "执行停止动作");
                    motion_home();
                    break;
                    
                case MOTION_WAVE_HAND:
                    ESP_LOGI(TAG, ">>> 执行挥手动作 <<<");
                    motion_wave_hand();
                    ESP_LOGI(TAG, "挥手动作完成");
                    break;
                    
                case MOTION_MOVE_EAR:
                    ESP_LOGI(TAG, ">>> 执行动耳朵动作 <<<");
                    motion_move_ear();
                    ESP_LOGI(TAG, "动耳朵动作完成");
                    break;
                    
                default:
                    ESP_LOGW(TAG, "未知指令: %d", received_cmd.motion_type);
                    break;
            }
        }
    }
}
   
    
   


