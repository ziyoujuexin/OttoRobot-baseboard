#ifndef SDCARD_ANIMATION_PROVIDER_H
#define SDCARD_ANIMATION_PROVIDER_H

#include "AnimationProvider.h"

/**
 * @class SDCardAnimationProvider
 * @brief Provides animation data by loading files from an SD card into PSRAM.
 */
class SDCardAnimationProvider : public AnimationProvider {
public:
    explicit SDCardAnimationProvider(const std::string& base_path = "/sdcard/animations");

    AnimationData getAnimationData(const std::string& animation_name) override;
    void releaseAnimationData(AnimationData& anim_data) override;

private:
    std::string m_base_path;
};

#endif // SDCARD_ANIMATION_PROVIDER_H
