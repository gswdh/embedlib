# AR0822 Library
# CMakeLists.txt for ar0822 library

# Create a static library target
add_library(ar0822 STATIC
    ar0822.c
)

# Set include directories for this library
target_include_directories(ar0822 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(ar0822 PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(ar0822 PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(ar0822 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 