# TMC2209 Stepper Motor Driver Library
# CMakeLists.txt for tmc2209 library

# Create a static library target
add_library(tmc2209 STATIC
    tmc2209.c
    tmc2209_example.c
    tmc2209_test.c
)

# Set include directories for this library
target_include_directories(tmc2209 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(tmc2209 PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(tmc2209 PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(tmc2209 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 