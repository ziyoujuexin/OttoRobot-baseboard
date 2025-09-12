#ifndef SD_CARD_MANAGER_H
#define SD_CARD_MANAGER_H

#include "esp_err.h"

namespace sd_card_manager {

/**
 * @brief Initializes and mounts the SD card to the VFS.
 * 
 * This function handles the configuration of the SDMMC host and slot,
 * and mounts the FAT filesystem at the specified base path.
 * 
 * This should be called once at application startup.
 * 
 * @param mount_path The path in the VFS where the SD card will be mounted (e.g., "/sdcard").
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t init(const char* mount_path = "/sdcard");

/**
 * @brief Deinitializes and unmounts the SD card.
 * 
 * @return ESP_OK on success, or an error code otherwise.
 */
esp_err_t deinit();

} // namespace sd_card_manager

#endif // SD_CARD_MANAGER_H
