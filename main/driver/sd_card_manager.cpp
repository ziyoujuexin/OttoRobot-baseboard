#include "sd_card_manager.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"

static const char* TAG = "SD_CARD_MANAGER";
static sdmmc_card_t* s_card = nullptr;
static const char* s_mount_path = nullptr;

esp_err_t sd_card_manager::init(const char* mount_path) {
    ESP_LOGI(TAG, "Initializing SD card...");

    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false,
        .use_one_fat = false,
    };

    // Use SDMMC peripheral to communicate with SD card.
    ESP_LOGI(TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = (gpio_num_t)43;
    slot_config.cmd = (gpio_num_t)44;
    slot_config.d0 = (gpio_num_t)39;
    slot_config.d1 = (gpio_num_t)40;
    slot_config.d2 = (gpio_num_t)41;
    slot_config.d3 = (gpio_num_t)42;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ret = esp_vfs_fat_sdmmc_mount(mount_path, &host, &slot_config, &mount_config, &s_card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set format_if_mount_failed = true.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors.", esp_err_to_name(ret));
        }
        return ret;
    }

    s_mount_path = mount_path;
    ESP_LOGI(TAG, "SD card mounted successfully at %s", mount_path);

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, s_card);

    return ESP_OK;
}

esp_err_t sd_card_manager::deinit() {
    if (s_mount_path == nullptr || s_card == nullptr) {
        ESP_LOGW(TAG, "SD card not initialized or already deinitialized.");
        return ESP_OK;
    }
    
    esp_err_t ret = esp_vfs_fat_sdcard_unmount(s_mount_path, s_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unmount SD card (%s).", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SD card unmounted successfully.");
    }
    
    s_card = nullptr;
    s_mount_path = nullptr;
    
    return ret;
}
