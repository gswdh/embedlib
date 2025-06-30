# MAX17049 Fuel Gauge Library
# CMakeLists.txt for max17049 library

# Create a static library target
add_library(max17049 STATIC
    max17049.c
)

# Set include directories for this library
target_include_directories(max17049 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(max17049 PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(max17049 PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(max17049 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 