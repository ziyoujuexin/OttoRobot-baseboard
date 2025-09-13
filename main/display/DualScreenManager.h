#ifndef DUAL_SCREEN_MANAGER_H
#define DUAL_SCREEN_MANAGER_H

#include "lvgl.h"
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

    void DisplayAnimation(ScreenId screen, const std::string& anim_path);

    void ClearScreen(ScreenId screen);

private:
    lv_obj_t* m_left_anim_obj;
    lv_obj_t* m_right_anim_obj;

    void create_anim_obj(lv_obj_t** obj_ptr, const std::string& anim_path);
};

#endif // DUAL_SCREEN_MANAGER_H
