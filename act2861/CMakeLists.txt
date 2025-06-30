# ACT2861 Library
# CMakeLists.txt for act2861 library

# Create a static library target
add_library(act2861 STATIC
    act2861.c
)

# Set include directories for this library
target_include_directories(act2861 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(act2861 PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(act2861 PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(act2861 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 