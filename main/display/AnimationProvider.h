#ifndef ANIMATION_PROVIDER_H
#define ANIMATION_PROVIDER_H

#include <string>

/**
 * @class AnimationProvider
 * @brief Abstract base class for providing animation resource paths.
 */
class AnimationProvider {
public:
    virtual ~AnimationProvider() = default;

    /**
     * @brief Gets the file path for a given animation name.
     * @param animation_name The logical name of the animation (e.g., "happy").
     * @return A string representing the full path to the animation file.
     *         Returns an empty string if the animation is not found.
     */
    virtual std::string getAnimationPath(const std::string& animation_name) = 0;
};

#endif // ANIMATION_PROVIDER_H
