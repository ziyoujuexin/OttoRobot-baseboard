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
    AnimationPlayer(AnimationManager* anim_manager, DualScreenManager* display_manager, QueueHandle_t ui_command_queue);
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
    DualScreenManager* m_display_manager; // Kept for now as per main.cpp, but logic will be removed
    QueueHandle_t m_ui_command_queue; // Handle to the central UI command queue
    
    TaskHandle_t m_task_handle = nullptr;

    // State for cyclical animation playback
    std::string m_current_anim_name;
    std::string m_next_anim_name;
    SemaphoreHandle_t m_next_anim_mutex = nullptr;

};

#endif // ANIMATION_PLAYER_H
