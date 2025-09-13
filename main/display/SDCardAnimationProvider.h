#ifndef SDCARD_ANIMATION_PROVIDER_H
#define SDCARD_ANIMATION_PROVIDER_H

#include "AnimationProvider.h"

/**
 * @class SDCardAnimationProvider
 * @brief Provides animation paths by checking for files on an SD card.
 */
class SDCardAnimationProvider : public AnimationProvider {
public:
    explicit SDCardAnimationProvider(const std::string& base_path = "/sdcard/animations");

    std::string getAnimationPath(const std::string& animation_name) override;

private:
    std::string m_base_path;
};

#endif // SDCARD_ANIMATION_PROVIDER_H
