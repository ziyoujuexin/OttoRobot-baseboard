#ifndef ANIMATION_DATA_H
#define ANIMATION_DATA_H

#include <vector>
#include <cstdint>
#include <string>

/**
 * @struct AnimationData
 * @brief Holds the binary data for an animation.
 *
 * An instance of this struct represents a loaded animation resource (e.g., a GIF file's content).
 * It uses a std::vector to automatically manage the memory for the data buffer.
 */
struct AnimationData {
    std::vector<uint8_t> data; // Buffer containing the animation data
    std::string name;          // The original name of the animation, for debugging

    /**
     * @brief Checks if the animation data is valid (not empty).
     * @return True if data is present, false otherwise.
     */
    bool IsValid() const {
        return !data.empty();
    }
};

#endif // ANIMATION_DATA_H
