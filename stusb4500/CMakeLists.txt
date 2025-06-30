# STUSB4500 USB Power Delivery Library
# CMakeLists.txt for stusb4500 library

# Create a static library target
add_library(stusb4500 STATIC
    stusb4500.c
)

# Set include directories for this library
target_include_directories(stusb4500 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(stusb4500 PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(stusb4500 PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(stusb4500 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 