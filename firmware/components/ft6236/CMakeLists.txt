set(srcs "src/ft6236.c")
set(includes "include")
set(publics_requires "driver" "esp_lcd" "esp_lcd_touch")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${includes}"
                       REQUIRES "${publics_requires}")