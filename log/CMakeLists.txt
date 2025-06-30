# Log Library
# CMakeLists.txt for log library

# Create a static library target
add_library(log STATIC
    log.c
)

# Set include directories for this library
target_include_directories(log PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(log PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(log PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(log PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 