# Log library
add_library(log STATIC
    log.c
)

# Include directories
target_include_directories(log PUBLIC
    .
)

# Set properties to ensure proper linking
set_target_properties(log PROPERTIES
    POSITION_INDEPENDENT_CODE OFF
    C_VISIBILITY_PRESET hidden
)

# Ensure weak symbols are properly handled
target_compile_options(log PRIVATE
    -fvisibility=hidden
) 