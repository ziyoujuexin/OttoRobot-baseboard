#ifndef ANIMATION_MANAGER_H
#define ANIMATION_MANAGER_H

#include <string>
#include <memory>
#include "AnimationProvider.h"
#include "DualScreenManager.h"

class AnimationManager {
public:
    AnimationManager(std::unique_ptr<AnimationProvider> provider, DualScreenManager* display_manager);

    bool PlayAnimation(const std::string& animation_name, ScreenId screen);
    bool StopAnimation(); // Placeholder for future use
    bool PauseAnimation(); // Placeholder for future use
    bool ResumeAnimation(); // Placeholder for future use
    bool ClearAnimation(); // Placeholder for future use

private:
    std::unique_ptr<AnimationProvider> m_provider;
    DualScreenManager* m_display_manager;
    std::string m_current_lvgl_path; // 持有当前的动画路径，防止悬空指针
};

#endif // ANIMATION_MANAGER_H
