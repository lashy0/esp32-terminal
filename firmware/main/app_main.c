#include <stdio.h>

#include "esp_flash.h"
#include "esp_system.h"
#include "esp_chip_info.h"

void app_main(void)
{
    printf("Esp-IDF version: %s\n", esp_get_idf_version());

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    printf("This is ESP32 chip with %d CPU cores\n", chip_info.cores);

    uint32_t flash_size = 0;
    if (esp_flash_get_physical_size(NULL, &flash_size) == ESP_OK) {
        printf("Physical flash size: %ld MB\n", flash_size / (1024 * 1024));
    } else {
        printf("Failed to get physical flash size!\n");
    }
    printf("Free heap: %ld bytes\n", esp_get_free_heap_size());
}
