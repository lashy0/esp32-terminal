set(publics_requires "bt" "nvs_flash" "esp_lcd" "ili9488" "lcd_backlight" "ft6236")

idf_component_register(SRCS "app_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES "${publics_requires}")