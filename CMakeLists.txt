cmake_minimum_required(VERSION 3.12)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(eeio C CXX ASM)

pico_sdk_init()

add_executable(eeio
        main.c
        eeio_proto.c
        at24cx.c
        )

target_compile_options(eeio PRIVATE -Wall -Wextra -Wno-unused-parameter)

target_link_libraries(eeio
        pico_stdlib
        hardware_i2c
)

# Uncomment (or pass -DEEIO_DEBUG=1 to cmake) for protocol/USB debug on UART0
if (EEIO_DEBUG)
    target_compile_definitions(eeio PRIVATE EEIO_DEBUG=1)
endif()

pico_add_extra_outputs(eeio)

pico_enable_stdio_uart(eeio 1)
pico_enable_stdio_usb(eeio 0)
