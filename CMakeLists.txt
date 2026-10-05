cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD spotpear_rp2350b_mini_a CACHE STRING "RP2350B board")
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards CACHE STRING "Custom board headers")
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(snes_rp2350b_capture C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/third_party/cx4.c" OR
   NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/third_party/cx4.h")
  message(FATAL_ERROR "CX4 LLE ausente. Rode PREPARAR_E_COMPILAR_CX4_v0.4.2.ps1 ou use o GitHub Actions desta versao.")
endif()

add_executable(snes_rp2350b_capture
    main.c
    cx4_active_bus.c
    third_party/cx4.c
)
target_include_directories(snes_rp2350b_capture PRIVATE ${CMAKE_CURRENT_LIST_DIR}/third_party)

pico_generate_pio_header(snes_rp2350b_capture ${CMAKE_CURRENT_LIST_DIR}/capture_low.pio)
pico_generate_pio_header(snes_rp2350b_capture ${CMAKE_CURRENT_LIST_DIR}/capture_high.pio)
pico_generate_pio_header(snes_rp2350b_capture ${CMAKE_CURRENT_LIST_DIR}/capture_read_trigger.pio)
pico_generate_pio_header(snes_rp2350b_capture ${CMAKE_CURRENT_LIST_DIR}/capture_read_low.pio)
pico_generate_pio_header(snes_rp2350b_capture ${CMAKE_CURRENT_LIST_DIR}/capture_read_high.pio)

target_link_libraries(snes_rp2350b_capture
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_dma
    hardware_gpio
    m
)
pico_enable_stdio_usb(snes_rp2350b_capture 1)
pico_enable_stdio_uart(snes_rp2350b_capture 0)
target_compile_definitions(snes_rp2350b_capture PRIVATE PICO_PIO_USE_GPIO_BASE=1)
pico_add_extra_outputs(snes_rp2350b_capture)
