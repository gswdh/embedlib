# SY103 Library
# CMakeLists.txt for sy103 library

# Create a static library target
add_library(sy103 STATIC
    sy103.c
)

# Set include directories for this library
target_include_directories(sy103 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(sy103 PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(sy103 PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(sy103 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 