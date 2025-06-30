# TCA6408 I/O Expander Library
# CMakeLists.txt for tca6408 library

# Create a static library target
add_library(tca6408 STATIC
    tca6408.c
)

# Set include directories for this library
target_include_directories(tca6408 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(tca6408 PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(tca6408 PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(tca6408 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 