cmake_minimum_required(VERSION 3.13)

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
# Project
# ====================================================================================

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(drone C CXX ASM)

pico_sdk_init()

# ====================================================================================
# FreeRTOS
# ====================================================================================

include(FreeRTOS_Kernel_import.cmake)

# ====================================================================================
# Executable
# ====================================================================================

add_executable(drone
    drone.cpp
)

pico_set_program_name(drone "drone")
pico_set_program_version(drone "0.1")

pico_enable_stdio_usb(drone 1)
pico_enable_stdio_uart(drone 0)

# ====================================================================================
# ADXL345 Library
# ====================================================================================

add_library(ADXL345
    sensors/accelerometer/ADXL345.cpp
)

target_include_directories(ADXL345 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/sensors/accelerometer
)

target_link_libraries(ADXL345 PUBLIC
    pico_stdlib
    hardware_i2c
)

# ====================================================================================
# Servo ESC Library
# ====================================================================================

add_library(ServoESC
    motion/pwm/ServoESC.cpp
)

target_include_directories(ServoESC PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/motion/pwm
)

target_link_libraries(ServoESC PUBLIC
    pico_stdlib
    hardware_pwm
    hardware_clocks
)

# ====================================================================================
# IPwm Interface Library
# ====================================================================================

add_library(IPwm INTERFACE)

target_include_directories(IPwm INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Interfaces
)

# ====================================================================================
# Servo Library
# ====================================================================================

add_library(Servo
    Drivers/PWM/Servo.cpp
)

target_include_directories(Servo PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/PWM
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Interfaces
)

target_link_libraries(Servo PUBLIC
    pico_stdlib
    hardware_pwm
    hardware_clocks
    IPwm
)

# ====================================================================================
# Main Target
# ====================================================================================

target_include_directories(drone PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(drone PRIVATE
    pico_stdlib
    hardware_i2c
    hardware_pwm

    ADXL345
    ServoESC
    Servo

    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

# ====================================================================================
# Output
# ====================================================================================

pico_add_extra_outputs(drone)