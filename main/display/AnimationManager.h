#ifndef ANIMATION_MANAGER_H
#define ANIMATION_MANAGER_H

#include <string>
#include <memory>
#include "AnimationProvider.h"
#include "DualScreenManager.h"

class AnimationManager {
public:
    AnimationManager(std::unique_ptr<AnimationProvider> provider);

    std::string getAnimationPath(const std::string& animation_name);

private:
    std::unique_ptr<AnimationProvider> m_provider;
};

#endif // ANIMATION_MANAGER_H
