# STM32 Bootloader library
add_library(stm32boot STATIC
    stm32boot.c
)

# Include directories
target_include_directories(stm32boot PUBLIC
    .
)
