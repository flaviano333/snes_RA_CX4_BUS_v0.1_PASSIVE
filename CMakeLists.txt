cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD spotpear_rp2350b_mini_a CACHE STRING "RP2350B board")
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards CACHE STRING "Custom board headers")
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(cx4_early_read_lle_v8_3 C CXX ASM)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/third_party/cx4.c" OR
   NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/third_party/cx4.h")
  message(FATAL_ERROR "CX4 core missing. Build with the included GitHub Actions workflow.")
endif()

add_executable(cx4_early_read_lle_v8_3
    main.c
    cx4_active_bus.c
    third_party/cx4.c
)
target_include_directories(cx4_early_read_lle_v8_3 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/third_party)
target_link_libraries(cx4_early_read_lle_v8_3 pico_stdlib pico_multicore hardware_gpio hardware_clocks m)
pico_enable_stdio_usb(cx4_early_read_lle_v8_3 1)
pico_enable_stdio_uart(cx4_early_read_lle_v8_3 0)
target_compile_definitions(cx4_early_read_lle_v8_3 PRIVATE PICO_PIO_USE_GPIO_BASE=1)
target_compile_options(cx4_early_read_lle_v8_3 PRIVATE -O3 -ffunction-sections -fdata-sections)
pico_add_extra_outputs(cx4_early_read_lle_v8_3)
