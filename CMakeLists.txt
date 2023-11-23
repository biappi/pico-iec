cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico-iec)

pico_sdk_init()

add_executable(pico-iec
    pico-iec.c
)

target_link_libraries(pico-iec pico_stdlib)

pico_add_extra_outputs(pico-iec)
pico_enable_stdio_usb(pico-iec 1)
pico_enable_stdio_uart(pico-iec 0)
