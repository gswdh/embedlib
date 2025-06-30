# GMAX0505 Library
# CMakeLists.txt for gmax0505 library

# Create a static library target
add_library(gmax0505 STATIC
    gmax0505.c
)

# Set include directories for this library
target_include_directories(gmax0505 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(gmax0505 PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(gmax0505 PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(gmax0505 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 