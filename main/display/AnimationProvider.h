#ifndef ANIMATION_PROVIDER_H
#define ANIMATION_PROVIDER_H

#include <cstdint>
#include <string>

// Represents the raw data of a single animation file
struct AnimationData {
    const uint8_t* data = nullptr;
    uint32_t size = 0;
    bool is_valid = false;
};

// Represents a set of animations for the dual-screen setup
struct AnimationPair {
    AnimationData left_anim;
    AnimationData right_anim;
    bool is_mirrored = false; // True if left_anim should be used for both screens

    // Helper to check if the pair contains any valid data
    bool is_valid() const {
        return left_anim.is_valid || right_anim.is_valid;
    }
};

/**
 * @class AnimationProvider
 * @brief Abstract interface for providing animation data.
 */
class AnimationProvider {
public:
    virtual ~AnimationProvider() = default;

    virtual AnimationPair getAnimationData(const std::string& animation_name) = 0;
    virtual void releaseAnimationData(AnimationData& anim_data) = 0;
    // Add a helper to release a pair
    virtual void releaseAnimationPair(AnimationPair& anim_pair) {
        releaseAnimationData(anim_pair.left_anim);
        // Only release right if it's not mirrored (i.e., not pointing to the same data)
        if (!anim_pair.is_mirrored) {
            releaseAnimationData(anim_pair.right_anim);
        }
    }
};

#endif // ANIMATION_PROVIDER_H