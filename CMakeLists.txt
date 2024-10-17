cmake_minimum_required(VERSION 3.13)

# Detect if the active kit is an ARM cross-compiler
if(CMAKE_CXX_COMPILER MATCHES "arm-none-eabi")
    message(STATUS "Detected that the current kit is a cross-compiler.")
    set(CrossCompiling 1)
else()
    message(STATUS "Detected that the current kit is a host compiler. Building the test harness.")
endif()

# Detect if the active kit is an ARM cross-compiler
if(CrossCompiling)
    # Yes, build for the RP2040

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)

    # Specify an increased oscillator startup delay to work around a potential hardware issue where the 
    # clock is slow to boot.
    add_compile_definitions(PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)

    # Set the path to the Pico SDK
    set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk")
    set(CMAKE_MAKE_PROGRAM "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/ninja/ninja.exe")

    # Pull in Raspberry Pi Pico SDK (must be before project)
    include(pico_sdk_import.cmake)

    project(cc3501-labs C CXX ASM)
    
    # Initialise the Raspberry Pi Pico SDK
    pico_sdk_init()

    add_executable(labs)
    target_sources(labs 
        PUBLIC
        src/main.cpp
        src/json.hpp
        src/ICM42688.hpp
        src/ICM42688.cpp
        src/drivers/logging/logging.cpp
    )
    target_include_directories(labs
        PUBLIC 
        src/
    )

    pico_set_program_name(labs "cc3501-labs")
    pico_set_program_version(labs "0.2")

    # select UART for standard IO
    pico_enable_stdio_uart(labs 1)
    pico_enable_stdio_usb(labs 1)

    # compile the PIO file

    # Add the standard library to the build
    target_link_libraries(labs
        pico_stdlib
        hardware_spi
        hardware_i2c
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_timer
        hardware_watchdog
        hardware_clocks
        hardware_pwm
        pico_multicore
        # pico_util_queue
    )

    pico_add_extra_outputs(labs)

else()

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 20)

    # We are building natively, so create the test harness instead
    project(cc3501-labs CXX)

    add_executable(labs)
    target_sources(labs 
        PUBLIC
        src/main.cpp
        src/json.hpp
        src/ICM42688.hpp
        src/ICM42688.cpp
        src/drivers/logging/logging.cpp
        tests/mocks/pico/stdlib.cpp
        tests/mocks/pico/time.cpp
        tests/mocks/hardware/gpio.cpp
        tests/mocks/hardware/pio.cpp
    )
    target_include_directories(labs
        PUBLIC 
        src/
        tests/
        tests/mocks/
    )
    target_compile_definitions(labs 
        PUBLIC
        TEST_HARNESS=1
    )

endif()

target_compile_definitions(labs 
    PUBLIC
    LOG_DRIVER_STYLE=${LogDriverImplementation}
)
