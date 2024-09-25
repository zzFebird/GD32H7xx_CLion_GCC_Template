add_library(drivers)

target_include_directories(drivers PUBLIC
    ./
    ./CMSIS
    ./CMSIS/GD/GD32H7xx/Include
    ./GD32H7xx_standard_peripheral/Include
)

file(GLOB_RECURSE DRIV_SRC
    ./GD32H7xx_standard_peripheral/*.*
    ./SYSTEM/*.*
)

list(APPEND DRIV_SRC ./CMSIS/GD/GD32H7xx/Source/ARM/startup_gd32h7xx.s)
list(APPEND DRIV_SRC ./CMSIS/GD/GD32H7xx/Source/system_gd32h7xx.c)

target_sources(drivers PUBLIC ${DRIV_SRC})