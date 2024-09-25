add_library(drivers)

target_include_directories(drivers PUBLIC
    ./
    ./CMSIS
    ./CMSIS/GD/GD32H7xx/Include
    ./GD32H7xx_standard_peripheral/Include
)

file(GLOB_RECURSE START_SRC
    ./CMSIS/GD/GD32H7xx/Source/ARM/startup_gd32h7xx.s
    ./CMSIS/GD/GD32H7xx/Source/system_gd32h7xx.c
)

file(GLOB_RECURSE DRIV_SRC
    ./GD32H7xx_standard_peripheral/*.c
    ./SYSTEM/*.c
)

file(GLOB_RECURSE BSP_SRC
    ./BSP/KEY/*.c
    ./BSP/LED/*.c
    ./BSP/MPU/*.c
    ./BSP/NORFLASH/*.c
    ./BSP/QSPI/*.c
    ./BSP/SDIO/*.c
    ./BSP/SDRAM/*.c
)

target_sources(drivers PUBLIC ${START_SRC} ${DRIV_SRC} ${BSP_SRC})