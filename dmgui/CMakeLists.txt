# DMGUI Library
# CMakeLists.txt for dmgui library

# Create a static library target
add_library(dmgui STATIC
    dmgui.c
)

# Set include directories for this library
target_include_directories(dmgui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(dmgui PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(dmgui PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(dmgui PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 