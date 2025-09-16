#ifndef LVGL_FS_PORT_H
#define LVGL_FS_PORT_H

/**
 * @brief 初始化并注册 LVGL 的文件系统驱动.
 * 该驱动将 'S' 盘符映射到 ESP-IDF 的 VFS.
 */
void lvgl_fs_driver_init(void);

#endif // LVGL_FS_PORT_H