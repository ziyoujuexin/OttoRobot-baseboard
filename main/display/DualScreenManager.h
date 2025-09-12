#ifndef DUAL_SCREEN_MANAGER_H
#define DUAL_SCREEN_MANAGER_H

#include "lvgl.h"
#include "AnimationData.h"

enum ScreenId {
    SCREEN_LEFT,
    SCREEN_RIGHT,
    SCREEN_BOTH
};

/**
 * @class DualScreenManager
 * @brief Low-level controller for the dual eye screens.
 *
 * This class is responsible for the direct manipulation of LVGL objects on the screens.
 * It displays animation data provided to it, but has no knowledge of where the data comes from.
 */
class DualScreenManager {
public:
    DualScreenManager();
    ~DualScreenManager();

    // Delete copy and assignment operators
    DualScreenManager(const DualScreenManager&) = delete;
    DualScreenManager& operator=(const DualScreenManager&) = delete;

    /**
     * @brief Displays an animation from its binary data on a specific screen.
     * @param screen The target screen(s).
     * @param anim_data The animation data to display.
     */
    void DisplayAnimation(ScreenId screen, const AnimationData& anim_data);

    void ClearScreen(ScreenId screen);

private:
    // LVGL objects for the animations on each screen
    lv_obj_t* m_left_anim_obj;
    lv_obj_t* m_right_anim_obj;

    void create_anim_obj(lv_obj_t** obj_ptr, const AnimationData& anim_data);
};

#endif // DUAL_SCREEN_MANAGER_H
