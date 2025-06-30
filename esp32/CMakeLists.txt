# ESP32 Library
# CMakeLists.txt for esp32 library

# Create a static library target
add_library(esp32 STATIC
    esp32.c
)

# Set include directories for this library
target_include_directories(esp32 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(esp32 PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(esp32 PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(esp32 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 