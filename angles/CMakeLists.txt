# Angles Library
# CMakeLists.txt for angles library

# Create a static library target
add_library(angles STATIC
    angles.c
)

# Set include directories for this library
target_include_directories(angles PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(angles PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(angles PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(angles PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 