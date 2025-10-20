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
    QueueHandle_t m_player_queue = nullptr; // Internal queue for one-shot commands

    // State machine for animation playback
    enum class PlayerState { PLAYING_DEFAULT, PLAYING_ONESHOT };
    PlayerState m_current_state = PlayerState::PLAYING_DEFAULT;
    std::string m_current_anim_name = "eyec";
    TickType_t m_one_shot_start_time = 0;
    float m_time_scale_percent = 1.1f;
};

#endif // ANIMATION_PLAYER_H
