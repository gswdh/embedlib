# TMC2209 library
add_library(tmc2209 STATIC
    tmc2209.c
)

# Include directories
target_include_directories(tmc2209 PUBLIC
    .
) 