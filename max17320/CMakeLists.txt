# MAX17320 Fuel Gauge Library
# CMakeLists.txt for max17320 library

# Create a static library target
add_library(max17320 STATIC
    max17320.c
)

# Set include directories for this library
target_include_directories(max17320 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(max17320 PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(max17320 PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(max17320 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 