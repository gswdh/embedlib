# SSD1309Z OLED Display Library
# CMakeLists.txt for ssd1309z library

# Create a static library target
add_library(ssd1309z STATIC
    ssd1309z.c
)

# Set include directories for this library
target_include_directories(ssd1309z PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(ssd1309z PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(ssd1309z PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(ssd1309z PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 