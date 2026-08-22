# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
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
# Pico VS Code Extension
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

if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()

# ====================================================================================
# Pico
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
# Common Include Directories
# ====================================================================================

set(DRIVERS_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers
)

set(INTERFACES_DIR
    ${DRIVERS_DIR}/Interfaces
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
# ADXL345
# ====================================================================================

add_library(ADXL345
    Drivers/Sensors/ADXL345/ADXL345.cpp
)

target_include_directories(ADXL345 PUBLIC
    ${DRIVERS_DIR}/Sensors/ADXL345
    ${INTERFACES_DIR}
)

target_link_libraries(ADXL345 PUBLIC
    II2c
)

# ====================================================================================
# BMP280
# ====================================================================================

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

# ====================================================================================
# GY271 / QMC5883
# ====================================================================================

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

# ====================================================================================
# ICM20948
# ====================================================================================

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

target_include_directories(SensorManager
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Managers/Sensors
)

target_link_libraries(SensorManager
    PUBLIC
        ICM20948
        GY271
        BMP280
        ADXL345
)

# ====================================================================================
# Main Executable
# ====================================================================================

add_executable(drone
    drone.cpp
)

target_include_directories(drone PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${DRIVERS_DIR}
    ${INTERFACES_DIR}
)

# ====================================================================================
# Main Target Dependencies
# ====================================================================================

target_link_libraries(drone PRIVATE

    # Pico
    pico_stdlib

    # Hardware
    hardware_i2c
    hardware_pwm
    hardware_uart

    # Drivers
    I2c
    Elrs
    Esc
    Servo

    # Sensors
    ADXL345
    BMP280
    GY271
    ICM20948

    #Managers
    SensorManager

    # FreeRTOS
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

# ====================================================================================
# USB / UART
# ====================================================================================

pico_enable_stdio_usb(drone 1)
pico_enable_stdio_uart(drone 0)

# ====================================================================================
# Program Information
# ====================================================================================

pico_set_program_name(drone "drone")
pico_set_program_version(drone "0.1")

# ====================================================================================
# Output
# ====================================================================================

pico_add_extra_outputs(drone)