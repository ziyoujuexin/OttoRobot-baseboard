#ifndef ANIMATION_PROVIDER_H
#define ANIMATION_PROVIDER_H

#include <string>
#include <cstddef> // For size_t
#include <cstdint> // For uint8_t

/**
 * @struct AnimationData
 * @brief Holds the raw data of an animation loaded into memory.
 */
struct AnimationData {
    const uint8_t* data = nullptr; // Pointer to the animation data in memory
    size_t size = 0;               // Size of the data in bytes
    bool is_valid = false;         // Flag to check if the data is valid
};

/**
 * @class AnimationProvider
 * @brief Abstract base class for providing animation data.
 */
class AnimationProvider {
public:
    virtual ~AnimationProvider() = default;

    /**
     * @brief Gets the animation data for a given animation name.
     * This method is responsible for loading the animation into memory.
     * @param animation_name The logical name of the animation (e.g., "happy").
     * @return An AnimationData struct containing the pointer to the data and its size.
     *         If loading fails, the is_valid flag in the struct will be false.
     */
    virtual AnimationData getAnimationData(const std::string& animation_name) = 0;

    /**
     * @brief Releases the memory allocated for the animation data.
     * @param anim_data The AnimationData struct whose memory should be freed.
     */
    virtual void releaseAnimationData(AnimationData& anim_data) = 0;
};

#endif // ANIMATION_PROVIDER_H
