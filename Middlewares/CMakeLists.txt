add_library(middlewares)

target_include_directories(middlewares PUBLIC ./)

file(GLOB_RECURSE MID_SRC
    ./FATFS/*.c
    ./MALLOC/*.c
)

target_sources(middlewares PUBLIC ${MID_SRC})

target_link_libraries(middlewares PRIVATE drivers)

add_subdirectory(USB)
target_link_libraries(middlewares PUBLIC usb)