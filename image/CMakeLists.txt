# Image Library
# CMakeLists.txt for image library

# Create a static library target
add_library(image STATIC
    image.c
)

# Set include directories for this library
target_include_directories(image PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Set compile definitions if needed
target_compile_definitions(image PRIVATE
    # Add any library-specific definitions here
)

# Set compile options
target_compile_options(image PRIVATE
    -Wall
    -Wextra
)

# Set C standard
set_target_properties(image PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
) 