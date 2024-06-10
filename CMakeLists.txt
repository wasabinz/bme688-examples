cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(bme688-example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(forced_mode
       forced_mode.c
       bme68x.c
       common.c
)

pico_enable_stdio_uart(forced_mode 1)
pico_add_extra_outputs(forced_mode)

target_link_libraries(forced_mode
        pico_stdlib
        pico_cyw43_arch_none
        hardware_i2c
        )

add_executable(parallel_mode
       parallel_mode.c
       bme68x.c
       common.c
)

pico_enable_stdio_uart(parallel_mode 1)
pico_add_extra_outputs(parallel_mode)

target_link_libraries(parallel_mode
        pico_stdlib
        pico_cyw43_arch_none
        hardware_i2c
        )

add_executable(self_test
       self_test.c
       bme68x.c
       common.c
)

pico_enable_stdio_uart(self_test 1)
pico_add_extra_outputs(self_test)

target_link_libraries(self_test
        pico_stdlib
        pico_cyw43_arch_none
        hardware_i2c
        )

add_executable(sequential_mode
       sequential_mode.c
       bme68x.c
       common.c
)

pico_enable_stdio_uart(sequential_mode 1)
pico_add_extra_outputs(sequential_mode)

target_link_libraries(sequential_mode
        pico_stdlib
        pico_cyw43_arch_none
        hardware_i2c
        )
