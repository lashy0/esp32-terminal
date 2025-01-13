set(srcs "src/lcd_backlight.c")
set(includes "include")
set(publics_requires "driver")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${includes}"
                       REQUIRES "${publics_requires}")