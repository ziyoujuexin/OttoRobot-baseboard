#ifndef ANIMATION_PROVIDER_H
#define ANIMATION_PROVIDER_H

#include <string>
#include "AnimationData.h"

/**
 * @class AnimationProvider
 * @brief Abstract base class for providing animation data.
 */
class AnimationProvider {
public:
    virtual ~AnimationProvider() = default;

    /**
     * @brief Gets the animation data for a given animation name.
     * @param animation_name The logical name of the animation (e.g., "happy").
     * @return An AnimationData struct. If the animation is not found, the struct's
     *         data buffer will be empty.
     */
    virtual AnimationData getAnimationData(const std::string& animation_name) = 0;
};

#endif // ANIMATION_PROVIDER_H
