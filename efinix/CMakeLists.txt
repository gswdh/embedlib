# Efinix Library
# CMakeLists.txt for efinix library

# Create a static library target
add_library(efinix STATIC
    efinix.c
)

# Set include directories for this library
target_include_directories(efinix PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(efinix PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(efinix PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(efinix PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 