#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "display/DualScreenManager.h"
#include "display/AnimationManager.h"

// Define the command structure for UI updates
struct UiCommand {
    char animation_name[32];
};

class UIManager {
public:
    UIManager(DualScreenManager* display_manager, AnimationManager* animation_manager);
    
    // Initializes mutex, queue, and starts the LVGL task
    void init();

    // Provides access to the command queue for other components
    QueueHandle_t get_command_queue() const;

private:
    // Static task function to run LVGL handler
    static void lvgl_task(void *pvParameter);

    DualScreenManager* m_display_manager;
    AnimationManager* m_animation_manager;
    
    QueueHandle_t m_ui_command_queue;
    SemaphoreHandle_t m_lvgl_mutex;
    TaskHandle_t m_lvgl_task_handle;
};
