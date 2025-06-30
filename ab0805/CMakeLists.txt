# AB0805 Library
# CMakeLists.txt for ab0805 library

# Create a static library target
add_library(ab0805 STATIC
    ab0805.c
)

# Set include directories for this library
target_include_directories(ab0805 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(ab0805 PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(ab0805 PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(ab0805 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 