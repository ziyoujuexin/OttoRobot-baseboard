#include "UIManager.hpp"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "lvgl.h"

static const char* TAG = "UIManager";

UIManager::UIManager(DualScreenManager* display_manager, AnimationManager* animation_manager)
    : m_display_manager(display_manager),
      m_animation_manager(animation_manager),
      m_ui_command_queue(NULL),
      m_lvgl_mutex(NULL),
      m_lvgl_task_handle(NULL) {}

void UIManager::init() {
    m_lvgl_mutex = xSemaphoreCreateMutex();
    m_ui_command_queue = xQueueCreate(10, sizeof(UiCommand));

    // Create the LVGL task, passing a pointer to this UIManager instance
    xTaskCreatePinnedToCore(
        lvgl_task, 
        "lvgl_task", 
        4096, 
        this, // Pass the instance pointer
        configMAX_PRIORITIES - 1, 
        &m_lvgl_task_handle, 
        0
    );
    ESP_LOGI(TAG, "UIManager initialized and LVGL task created.");
}

QueueHandle_t UIManager::get_command_queue() const {
    return m_ui_command_queue;
}

// This is the FreeRTOS task that runs the LVGL timer and processes UI commands.
void UIManager::lvgl_task(void *pvParameter) {
    ESP_LOGI(TAG, "LVGL task started");
    UIManager* self = (UIManager*)pvParameter;

    // Initialize LVGL objects in the context of the LVGL task
    self->m_display_manager->init();
    
    uint32_t task_delay_ms = 10;
    AnimationPair current_anim_pair; // Holds the currently loaded animation data

    // ESP_ERROR_CHECK(esp_task_wdt_delete(xTaskGetCurrentTaskHandle())); // Remove WDT for this task

    while(1) {
        // 1. Check and process UI commands (non-blocking)
        UiCommand received_cmd;
        if (xQueueReceive(self->m_ui_command_queue, &received_cmd, 0) == pdPASS) {
            ESP_LOGI(TAG, "LVGL task received command to play: %s", received_cmd.animation_name);
            
            // 1. Release previous animation data if it's valid
            if (current_anim_pair.is_valid()) {
                self->m_animation_manager->releaseAnimationPair(current_anim_pair);
            }

            // 2. Get new animation data from the provider
            current_anim_pair = self->m_animation_manager->getAnimationData(received_cmd.animation_name);

            // 3. Update the display with the new data (from memory)
            self->m_display_manager->UpdateAnimationSource(current_anim_pair);

            // Force a yield here to allow other tasks (like idle task for WDT) to run
            // before potentially processing another command from the queue immediately.
            // vTaskDelay(pdMS_TO_TICKS(10));
        }
        // static uint32_t time_next;
        // // 2. Standard LVGL handler loop
        // if(xSemaphoreTake(self->m_lvgl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        //     time_next = lv_timer_handler();
        //     vTaskDelay(pdMS_TO_TICKS(time_next));
        //     xSemaphoreGive(self->m_lvgl_mutex);
        // } else {
        //     ESP_LOGW(TAG, "LVGL mutex take timed out");
        // }
        // 2. Standard LVGL handler loop
        if(xSemaphoreTake(self->m_lvgl_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            lv_timer_handler_run_in_period(task_delay_ms);
            xSemaphoreGive(self->m_lvgl_mutex);
        } else {
            ESP_LOGW(TAG, "LVGL mutex take timed out");
        }
        // vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}
