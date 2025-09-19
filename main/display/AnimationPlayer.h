#ifndef ANIMATION_PLAYER_H
#define ANIMATION_PLAYER_H

#include <string>
#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "AnimationManager.h"
#include "DualScreenManager.h"

// Forward declaration
class AnimationManager;

class AnimationPlayer {
public:
    AnimationPlayer(AnimationManager* anim_manager, DualScreenManager* display_manager, SemaphoreHandle_t lvgl_mutex);
    ~AnimationPlayer();

    void start();
    void playOneShotAnimation(const std::string& animation_name);

private:
    // Task and queue management
    void player_task();
    static void player_task_wrapper(void* param) {
        static_cast<AnimationPlayer*>(param)->player_task();
    }

    AnimationManager* m_anim_manager;
    DualScreenManager* m_display_manager;
    SemaphoreHandle_t m_lvgl_mutex;
    
    TaskHandle_t m_task_handle = nullptr;
    QueueHandle_t m_player_queue = nullptr;
};

#endif // ANIMATION_PLAYER_H
