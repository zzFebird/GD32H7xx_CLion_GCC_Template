add_library(middlewares)

target_include_directories(middlewares PUBLIC ./)

file(GLOB_RECURSE MID_SRC
    ./FATFS/exfuns/*.c
    ./FATFS/source/*.c
    ./MALLOC/*.c
#    ./USMART/*.c
)

target_sources(middlewares PUBLIC ${MID_SRC})

target_link_libraries(middlewares PRIVATE drivers)