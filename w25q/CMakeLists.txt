# W25Q library
add_library(w25q STATIC
    w25q.c
)

# Include directories
target_include_directories(w25q PUBLIC
    .
)
