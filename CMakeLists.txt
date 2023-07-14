# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.21)

# initialize pico-sdk from GIT
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_FETCH_FROM_GIT on)
set(CMAKE_C_STANDARD 23)
set(CMAKE_CXX_STANDARD 11)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
# include($ENV{PICO_SDK_PATH}/../pico_extras/external/pico_extras_import.cmake)
include($ENV{PICO_SDK_PATH}/tools/CMakeLists.txt)
if (NOT TARGET spdif_rx)
    add_library(spdif_rx INTERFACE)

    # pico_generate_pio_header(spdif_rx ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_44100.pio)
    pico_generate_pio_header(spdif_rx ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_48000.pio)
    pico_generate_pio_header(spdif_rx ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_96000.pio)
    pico_generate_pio_header(spdif_rx ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_192000.pio)
    pico_generate_pio_header(spdif_rx ${CMAKE_CURRENT_LIST_DIR}/spdif_rx_capture.pio)

    add_custom_command(
        OUTPUT
            ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_44100.pio
            ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_48000.pio
            ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_96000.pio
            ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_192000.pio
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/spdif_rx.pio
        COMMAND "python"
            ${CMAKE_CURRENT_LIST_DIR}/scripts/gen_pio_by_freq.py
            ${CMAKE_CURRENT_LIST_DIR}/spdif_rx.pio
            ${CMAKE_CURRENT_BINARY_DIR}
            44100
            48000
            96000
            192000
    )

    target_sources(spdif_rx INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/spdif_rx.c
    )

    target_include_directories(spdif_rx INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
    )

    target_link_libraries(spdif_rx INTERFACE
        pico_stdlib
        hardware_dma
        hardware_pio
        hardware_irq
        # pico_audio_32b
    )
endif()

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(spdif C CXX ASM)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} 
    main.cpp
)

# # Create C header file with the name <pio program>.pio.h
# pico_generate_pio_header(${PROJECT_NAME}  
#         ${CMAKE_CURRENT_LIST_DIR}/spdif_rx_capture.pio
# )
# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

if (NOT TARGET spdif_rx)
    add_library(spdif_rx INTERFACE)

    # pico_generate_pio_header(spdif_rx ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_44100.pio)
    pico_generate_pio_header(spdif_rx ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_48000.pio)
    pico_generate_pio_header(spdif_rx ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_96000.pio)
    pico_generate_pio_header(spdif_rx ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_192000.pio)
    pico_generate_pio_header(spdif_rx ${CMAKE_CURRENT_LIST_DIR}/spdif_rx_capture.pio)

    add_custom_command(
        OUTPUT
            # ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_44100.pio
            ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_48000.pio
            ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_96000.pio
            ${CMAKE_CURRENT_BINARY_DIR}/spdif_rx_192000.pio
        DEPENDS ${CMAKE_CURRENT_LIST_DIR}/spdif_rx.pio
        COMMAND "python"
            ${CMAKE_CURRENT_LIST_DIR}/scripts/gen_pio_by_freq.py
            ${CMAKE_CURRENT_LIST_DIR}/spdif_rx.pio
            ${CMAKE_CURRENT_BINARY_DIR}
            # 44100
            48000
            96000
            192000
    )

    target_sources(spdif_rx INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/spdif_rx.c
    )

    target_include_directories(spdif_rx INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
    )

    target_link_libraries(spdif_rx INTERFACE
        pico_stdlib
        hardware_dma
        hardware_pio
        hardware_irq
        # pico_audio_32b
    )
endif()


# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_gpio
    hardware_irq
    hardware_i2c
    pico_rand
    pico_multicore
    spdif_rx
    hardware_interp
    pico_mem_ops
    pico_divider
    )

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

