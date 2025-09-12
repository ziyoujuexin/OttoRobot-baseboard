#ifndef ANIMATION_MANAGER_H
#define ANIMATION_MANAGER_H

#include <string>
#include <memory>
#include "AnimationProvider.h"
#include "DualScreenManager.h"

/**
 * @class AnimationManager
 * @brief High-level coordinator for playing animations.
 *
 * This class orchestrates the process of fetching animation data and displaying it.
 * It connects a data source (AnimationProvider) with a display controller (DualScreenManager).
 */
class AnimationManager {
public:
    AnimationManager(std::unique_ptr<AnimationProvider> provider, DualScreenManager* display_manager);

    /**
     * @brief Plays an animation on a specific screen by its logical name.
     * @param animation_name The name of the animation to play (e.g., "happy").
     * @param screen The target screen(s).
     * @return True if the animation was found and passed to the display manager, false otherwise.
     */
    bool PlayAnimation(const std::string& animation_name, ScreenId screen);

private:
    std::unique_ptr<AnimationProvider> m_provider;
    DualScreenManager* m_display_manager; // Assumes DualScreenManager is managed elsewhere
};

#endif // ANIMATION_MANAGER_H