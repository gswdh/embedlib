# Statistics Library
# CMakeLists.txt for statistics library

# Create a static library target
add_library(statistics STATIC
    stats.c
)

# Set include directories for this library
target_include_directories(statistics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(statistics PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(statistics PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(statistics PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 