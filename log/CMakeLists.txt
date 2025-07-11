# Log library
add_library(log STATIC
    log.c
)

# Include directories
target_include_directories(log PUBLIC
    .
) 