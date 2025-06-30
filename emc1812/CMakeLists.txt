# EMC1812 Library
# CMakeLists.txt for emc1812 library

# Create a static library target
add_library(emc1812 STATIC
    emc1812.c
)

# Set include directories for this library
target_include_directories(emc1812 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(emc1812 PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(emc1812 PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(emc1812 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 