#ifndef DUAL_SCREEN_MANAGER_H
#define DUAL_SCREEN_MANAGER_H

#include "lvgl.h"
#include "GC9A01_driver.hpp"
#include <string>

enum ScreenId {
    SCREEN_LEFT,
    SCREEN_RIGHT,
    SCREEN_BOTH
};

class DualScreenManager {
public:
    DualScreenManager();
    ~DualScreenManager();

    DualScreenManager(const DualScreenManager&) = delete;
    DualScreenManager& operator=(const DualScreenManager&) = delete;

    void UpdateAnimationSource(const std::string& anim_path);

    void ClearScreen(ScreenId screen);

    // Getter for the GIF object
    lv_obj_t* get_gif_obj(ScreenId screen) const;

private:
    lv_display_t* m_left_disp;
    lv_display_t* m_right_disp;
    lv_obj_t* m_left_gif_obj;
    lv_obj_t* m_right_gif_obj;

    void clear_disp(lv_display_t* disp);
};

#endif // DUAL_SCREEN_MANAGER_H
