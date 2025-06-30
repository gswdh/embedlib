# LIS3DSH Accelerometer Library
# CMakeLists.txt for lis3dsh library

# Create a static library target
add_library(lis3dsh STATIC
    lis3dsh.c
)

# Set include directories for this library
target_include_directories(lis3dsh PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(lis3dsh PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(lis3dsh PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(lis3dsh PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 