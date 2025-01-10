set(srcs "src/ili9488.c")
set(includes "include")
set(publics_requires "driver" "esp_lcd")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${includes}"
                       REQUIRES "${publics_requires}")