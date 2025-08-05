#include "app_process.h"
#include "motion_control.h"
#include "esp32c3_szp.h"
#include "esp_task_wdt.h"  // 添加看门狗头文件

static const char *TAG = "app_process";

void hardwire_init(){
    ESP_LOGI(TAG, "开始硬件初始化");
    
    // 添加看门狗保护
    esp_task_wdt_add(NULL);
    esp_task_wdt_reset();
    
    // 串口初始化
    ESP_LOGI(TAG, "初始化UART...");
    uart_init();
    esp_task_wdt_reset();
    
    // 初始化直接舵机控制
    ESP_LOGI(TAG, "初始化舵机PWM...");
    servo_pwm_init();
    esp_task_wdt_reset();
    
    // 将所有舵机设置到中心位置
    ESP_LOGI(TAG, "设置舵机到中心位置...");
    servo_home_position();
    esp_task_wdt_reset();
    
    // 移除看门狗保护
    esp_task_wdt_delete(NULL);
    
    ESP_LOGI(TAG, "硬件初始化完成");
}

void app_start(void){
    ESP_LOGI(TAG, "启动应用任务");
    
    // 创建串口接收任务
    BaseType_t ret = xTaskCreate(uart_receive_task, "uart_receive_task", 4096, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "UART接收任务创建失败");
        return;
    }
    
    ESP_LOGI(TAG, "应用任务启动完成");
}