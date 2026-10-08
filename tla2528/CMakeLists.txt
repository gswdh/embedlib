# TLA2528 ADC library
add_library(tla2528 STATIC
    tla2528.c
)

target_include_directories(tla2528 PUBLIC
    .
)
