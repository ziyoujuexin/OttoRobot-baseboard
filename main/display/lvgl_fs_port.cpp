#include "lvgl_fs_port.h"
#include "lvgl.h"
#include "esp_log.h"
#include <string>
#include <stdio.h>

static const char *TAG = "LVGL_FS";

// --- 回调函数定义 ---

/**
 * @brief 打开文件.
 * @param drv LVGL 文件系统驱动指针.
 * @param path 文件路径 (例如, "S:/path/to/file.txt").
 * @param mode 打开模式 (LV_FS_MODE_RD | LV_FS_MODE_WR).
 * @return 成功时返回文件指针, 失败时返回 NULL.
 */
static void* fs_open_cb(lv_fs_drv_t* drv, const char* path, lv_fs_mode_t mode) {
    const char* posix_mode;
    if (mode == LV_FS_MODE_WR) posix_mode = "wb";
    else if (mode == LV_FS_MODE_RD) posix_mode = "rb";
    else if (mode == (LV_FS_MODE_WR | LV_FS_MODE_RD)) posix_mode = "rb+";
    else return NULL;

    // 创建VFS基础路径
    std::string vfs_path = "/sdcard";

    // 安全地处理LVGL路径
    const char* path_without_drive = path;
    // 检查路径是否包含盘符 (例如 'S:')
    if (path[0] != '\0' && path[1] == ':') {
        path_without_drive += 2; // 如果有，则跳过盘符
    }

    vfs_path += path_without_drive;

    FILE* f = fopen(vfs_path.c_str(), posix_mode);
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file: %s", vfs_path.c_str());
    }
    // (可以去掉成功日志，避免刷屏)
    // else {
    //     ESP_LOGI(TAG, "Successfully opened file: %s", vfs_path.c_str());
    // }
    return f;
}

/**
 * @brief 关闭文件.
 * @param drv LVGL 文件系统驱动指针.
 * @param file_p 文件指针.
 * @return 总是返回 LV_FS_RES_OK.
 */
static lv_fs_res_t fs_close_cb(lv_fs_drv_t* drv, void* file_p) {
    fclose((FILE*)file_p);
    return LV_FS_RES_OK;
}

/**
 * @brief 读取文件.
 * @param drv LVGL 文件系统驱动指针.
 * @param file_p 文件指针.
 * @param buf 存储读取数据的缓冲区.
 * @param btr 要读取的字节数.
 * @param br 实际读取的字节数 (输出).
 * @return LV_FS_RES_OK 表示成功, 其他表示失败.
 */
static lv_fs_res_t fs_read_cb(lv_fs_drv_t* drv, void* file_p, void* buf, uint32_t btr, uint32_t* br) {
    *br = fread(buf, 1, btr, (FILE*)file_p);
    ESP_LOGI(TAG, "fs_read_cb: requested=%d, read=%d", btr, *br);
    // fread 在文件末尾或出错时返回的值可能小于 btr，这是正常行为
    return LV_FS_RES_OK;
}

/**
 * @brief 设置文件读写位置.
 * @param drv LVGL 文件系统驱动指针.
 * @param file_p 文件指针.
 * @param pos 要移动到的位置.
 * @param whence 起始位置 (LV_FS_SEEK_SET/CUR/END).
 * @return LV_FS_RES_OK 表示成功.
 */
static lv_fs_res_t fs_seek_cb(lv_fs_drv_t* drv, void* file_p, uint32_t pos, lv_fs_whence_t whence) {
    int seek_mode;
    const char* whence_str;

    switch(whence) {
        case LV_FS_SEEK_SET: 
            seek_mode = SEEK_SET;
            whence_str = "SET";
            break;
        case LV_FS_SEEK_CUR:
            seek_mode = SEEK_CUR;
            whence_str = "CUR";
            break;
        case LV_FS_SEEK_END:
            seek_mode = SEEK_END;
            whence_str = "END";
            break;
        default:
            ESP_LOGE(TAG, "fs_seek_cb: Invalid whence: %d", whence);
            return LV_FS_RES_INV_PARAM;
    }

    ESP_LOGI(TAG, "fs_seek_cb: seeking to pos=%d, whence=%s", pos, whence_str);
    
    // fseek 成功返回0, 失败返回非0
    if (fseek((FILE*)file_p, pos, seek_mode) != 0) {
        ESP_LOGE(TAG, "fs_seek_cb: fseek failed!");
        return LV_FS_RES_UNKNOWN;
    }

    return LV_FS_RES_OK;
}

/**
 * @brief 获取当前文件读写位置.
 * @param drv LVGL 文件系统驱动指针.
 * @param file_p 文件指针.
 * @param pos_p 存储当前位置的指针 (输出).
 * @return LV_FS_RES_OK 表示成功.
 */
static lv_fs_res_t fs_tell_cb(lv_fs_drv_t* drv, void* file_p, uint32_t* pos_p) {
    // ftell 成功返回当前位置, 失败返回 -1L
    long pos = ftell((FILE*)file_p);
    if (pos == -1L) {
        ESP_LOGE(TAG, "fs_tell_cb: ftell failed!");
        *pos_p = 0; // 返回一个安全值
        return LV_FS_RES_UNKNOWN;
    }

    *pos_p = (uint32_t)pos;
    ESP_LOGI(TAG, "fs_tell_cb: current position is %d", *pos_p);
    return LV_FS_RES_OK;
}


// --- 驱动初始化函数 ---

void lvgl_fs_driver_init(void) {
    // 1. 创建一个静态的驱动变量 (必须是静态或全局的)
    static lv_fs_drv_t fs_drv;

    // 2. 初始化驱动变量
    lv_fs_drv_init(&fs_drv);

    // 3. 设置基本参数和回调函数
    fs_drv.letter = 'S';          // 盘符
    fs_drv.open_cb = fs_open_cb;
    fs_drv.close_cb = fs_close_cb;
    fs_drv.read_cb = fs_read_cb;
    fs_drv.seek_cb = fs_seek_cb;
    fs_drv.tell_cb = fs_tell_cb;

    // 4. GIF解码器不需要目录操作, 设为 NULL
    fs_drv.dir_open_cb = NULL;
    fs_drv.dir_read_cb = NULL;
    fs_drv.dir_close_cb = NULL;

    // 5. 注册驱动
    lv_fs_drv_register(&fs_drv);
    ESP_LOGI(TAG, "LVGL file system driver for 'S:' has been registered.");
}