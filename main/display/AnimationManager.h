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

private:
    std::unique_ptr<AnimationProvider> m_provider;
    DualScreenManager* m_display_manager;
};

#endif // ANIMATION_MANAGER_H
