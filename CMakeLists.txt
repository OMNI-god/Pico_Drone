# ====================================================================================
# Raspberry Pi Pico VS Code Extension
# DO NOT EDIT THE FOLLOWING LINES
# ====================================================================================

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)

set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)

if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()


# ====================================================================================
# CMake
# ====================================================================================

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# ====================================================================================
# Raspberry Pi Pico
# ====================================================================================

set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(drone C CXX ASM)

pico_sdk_init()


# ====================================================================================
# FreeRTOS
# ====================================================================================

include(FreeRTOS_Kernel_import.cmake)


# ====================================================================================
# Common Directories
# ====================================================================================

set(DRIVERS_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers
)

set(INTERFACES_DIR
    ${DRIVERS_DIR}/Interfaces
)

set(FLIGHT_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/Flight
)

set(MANAGERS_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/Managers
)


# ====================================================================================
# Interfaces
# ====================================================================================

# ------------------------------------------------------------------------------------
# IPwm
# ------------------------------------------------------------------------------------

add_library(IPwm INTERFACE)

target_include_directories(IPwm INTERFACE
    ${INTERFACES_DIR}
)


# ------------------------------------------------------------------------------------
# II2c
# ------------------------------------------------------------------------------------

add_library(II2c INTERFACE)

target_include_directories(II2c INTERFACE
    ${INTERFACES_DIR}
)


# ------------------------------------------------------------------------------------
# IUart
# ------------------------------------------------------------------------------------

add_library(IUart INTERFACE)

target_include_directories(IUart INTERFACE
    ${INTERFACES_DIR}
)


# ====================================================================================
# I2C Driver
# ====================================================================================

add_library(I2c
    Drivers/I2C/I2c.cpp
)

target_include_directories(I2c PUBLIC
    ${DRIVERS_DIR}/I2C
    ${INTERFACES_DIR}
)

target_link_libraries(I2c PUBLIC
    pico_stdlib
    hardware_i2c
    hardware_gpio
    II2c
)


# ====================================================================================
# UART / ELRS Driver
# ====================================================================================

add_library(Elrs
    Drivers/UART/Elrs.cpp
)

target_include_directories(Elrs PUBLIC
    ${DRIVERS_DIR}/UART
    ${INTERFACES_DIR}
)

target_link_libraries(Elrs PUBLIC
    pico_stdlib
    hardware_uart
    hardware_gpio
    IUart
)


# ====================================================================================
# PWM / ESC
# ====================================================================================

add_library(Esc
    Drivers/PWM/Esc.cpp
)

target_include_directories(Esc PUBLIC
    ${DRIVERS_DIR}/PWM
)

target_link_libraries(Esc PUBLIC
    pico_stdlib
    hardware_pwm
    hardware_clocks
    IPwm
)


# ====================================================================================
# PWM / Servo
# ====================================================================================

add_library(Servo
    Drivers/PWM/Servo.cpp
)

target_include_directories(Servo PUBLIC
    ${DRIVERS_DIR}/PWM
)

target_link_libraries(Servo PUBLIC
    pico_stdlib
    hardware_pwm
    hardware_clocks
    IPwm
)


# ====================================================================================
# Sensor Drivers
# ====================================================================================


# ------------------------------------------------------------------------------------
# ADXL345
# ------------------------------------------------------------------------------------

add_library(ADXL345
    Drivers/Sensors/ADXL345/ADXL345.cpp
)

target_include_directories(ADXL345 PUBLIC
    ${DRIVERS_DIR}/Sensors/ADXL345
    ${INTERFACES_DIR}
)

target_link_libraries(ADXL345 PUBLIC
    II2c
    pico_stdlib
)


# ------------------------------------------------------------------------------------
# BMP280
# ------------------------------------------------------------------------------------

add_library(BMP280
    Drivers/Sensors/BMP280/BMP280.cpp
)

target_include_directories(BMP280 PUBLIC
    ${DRIVERS_DIR}/Sensors/BMP280
    ${INTERFACES_DIR}
)

target_link_libraries(BMP280 PUBLIC
    II2c
    pico_stdlib
)


# ------------------------------------------------------------------------------------
# GY271 / QMC5883
# ------------------------------------------------------------------------------------

add_library(GY271
    Drivers/Sensors/GY271/GY271.cpp
)

target_include_directories(GY271 PUBLIC
    ${DRIVERS_DIR}/Sensors/GY271
    ${INTERFACES_DIR}
)

target_link_libraries(GY271 PUBLIC
    II2c
    pico_stdlib
)


# ------------------------------------------------------------------------------------
# ICM20948
# ------------------------------------------------------------------------------------

add_library(ICM20948
    Drivers/Sensors/ICM20948/ICM20948.cpp
)

target_include_directories(ICM20948 PUBLIC
    ${DRIVERS_DIR}/Sensors/ICM20948
    ${INTERFACES_DIR}
)

target_link_libraries(ICM20948 PUBLIC
    II2c
    pico_stdlib
)


# ====================================================================================
# Sensor Manager
# ====================================================================================

add_library(SensorManager
    Managers/Sensors/SensorManager.cpp
)

target_include_directories(SensorManager PUBLIC
    ${MANAGERS_DIR}/Sensors
)

target_link_libraries(SensorManager PUBLIC
    ICM20948
    GY271
    BMP280
    ADXL345
)


# ====================================================================================
# Flight / RC
# ====================================================================================

# ------------------------------------------------------------------------------------
# CRSF + RC Input + CRSF Telemetry
# ------------------------------------------------------------------------------------
#
# RC subsystem:
#
#   CrsfCrc
#       |
#       +--> CrsfParser
#       |
#       +--> CrsfTelemetry
#       |
#       +--> RCInput
#
# RCInput does NOT depend directly on Elrs.
#
# Elrs is responsible for UART transport.
# CrsfParser is responsible for CRSF protocol parsing.
# RCInput is responsible for channel processing.
# CrsfTelemetry is responsible for telemetry frame generation.
#
# ------------------------------------------------------------------------------------

add_library(RCInput

    Flight/RC/CrsfCrc.cpp
    Flight/RC/CrsfParser.cpp
    Flight/RC/CrsfTelemetry.cpp
    Flight/RC/RCConfig.cpp
    Flight/RC/RCInput.cpp
)

target_include_directories(RCInput PUBLIC
    ${FLIGHT_DIR}/RC
)

target_link_libraries(RCInput PUBLIC
    pico_stdlib
)


# ====================================================================================
# Main Executable
# ====================================================================================

add_executable(drone

    drone.cpp
)


# ====================================================================================
# Main Include Directories
# ====================================================================================

target_include_directories(drone PRIVATE

    ${CMAKE_CURRENT_SOURCE_DIR}

    ${DRIVERS_DIR}

    ${INTERFACES_DIR}

    ${FLIGHT_DIR}

    ${MANAGERS_DIR}
)


# ====================================================================================
# Main Target Dependencies
# ====================================================================================

target_link_libraries(drone PRIVATE

    # ----------------------------------------------------------------------------
    # Pico SDK
    # ----------------------------------------------------------------------------

    pico_stdlib


    # ----------------------------------------------------------------------------
    # Hardware
    # ----------------------------------------------------------------------------

    hardware_i2c

    hardware_pwm

    hardware_uart

    hardware_gpio

    hardware_clocks


    # ----------------------------------------------------------------------------
    # Interfaces
    # ----------------------------------------------------------------------------

    IPwm

    II2c

    IUart


    # ----------------------------------------------------------------------------
    # Drivers
    # ----------------------------------------------------------------------------

    I2c

    Elrs

    Esc

    Servo


    # ----------------------------------------------------------------------------
    # Flight
    # ----------------------------------------------------------------------------

    RCInput


    # ----------------------------------------------------------------------------
    # Sensors
    # ----------------------------------------------------------------------------

    ADXL345

    BMP280

    GY271

    ICM20948


    # ----------------------------------------------------------------------------
    # Managers
    # ----------------------------------------------------------------------------

    SensorManager


    # ----------------------------------------------------------------------------
    # FreeRTOS
    # ----------------------------------------------------------------------------

    FreeRTOS-Kernel

    FreeRTOS-Kernel-Heap4
)


# ====================================================================================
# USB / UART
# ====================================================================================

# USB serial enabled for debugging
pico_enable_stdio_usb(drone 1)

# Hardware UART stdio disabled
pico_enable_stdio_uart(drone 0)


# ====================================================================================
# Program Information
# ====================================================================================

pico_set_program_name(
    drone
    "drone"
)

pico_set_program_version(
    drone
    "0.1"
)


# ====================================================================================
# Extra Outputs
# ====================================================================================

pico_add_extra_outputs(drone)