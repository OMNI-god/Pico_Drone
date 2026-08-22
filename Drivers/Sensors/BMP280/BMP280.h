#pragma once

#include <cstdint>

#include "II2c.h"

class BMP280
{
public:
    // =========================================================================
    // Measurements
    // =========================================================================

    struct Measurements
    {
        float temperatureC = 0.0f;
        float pressurePa = 0.0f;
        float pressureHpa = 0.0f;
    };

public:
    explicit BMP280(
        II2c &bus,
        uint8_t address = 0x76);

    bool initialize();

    bool readTemperature(
        float &temperatureC);

    bool readPressure(
        float &pressurePa);

    bool read(
        Measurements &measurements);

    bool isConnected();

private:
    // =========================================================================
    // Registers
    // =========================================================================

    static constexpr uint8_t REG_CALIB =
        0x88;

    static constexpr uint8_t REG_ID =
        0xD0;

    static constexpr uint8_t REG_RESET =
        0xE0;

    static constexpr uint8_t REG_STATUS =
        0xF3;

    static constexpr uint8_t REG_CTRL_MEAS =
        0xF4;

    static constexpr uint8_t REG_CONFIG =
        0xF5;

    static constexpr uint8_t REG_DATA =
        0xF7;

    // =========================================================================
    // Device
    // =========================================================================

    static constexpr uint8_t DEVICE_ID =
        0x58;

    static constexpr uint8_t RESET_COMMAND =
        0xB6;

    // =========================================================================
    // STATUS
    // =========================================================================

    static constexpr uint8_t STATUS_MEASURING =
        0x08;

    static constexpr uint8_t STATUS_IM_UPDATE =
        0x01;

    // =========================================================================
    // CONFIG
    // =========================================================================
    //
    // t_sb   = 125 ms
    // filter = x4
    // spi3w  = disabled
    //
    // 010 010 00
    //  ^     ^
    //  |     filter
    //  standby
    //
    // = 0x48
    //
    // For forced mode, standby does not matter.
    // =========================================================================

    static constexpr uint8_t CONFIG_VALUE =
        (2u << 5) |
        (2u << 2);

    // =========================================================================
    // CTRL_MEAS
    // =========================================================================
    //
    // Temperature = x2
    // Pressure    = x16
    // Mode        = forced
    //
    // 010 101 01
    //
    // = 0x55
    // =========================================================================

    static constexpr uint8_t CTRL_MEAS_FORCED =
        (2u << 5) |
        (5u << 2) |
        1u;

    // =========================================================================
    // CTRL_MEAS SLEEP
    // =========================================================================

    static constexpr uint8_t CTRL_MEAS_SLEEP =
        (2u << 5) |
        (5u << 2) |
        0u;

private:
    // =========================================================================
    // I2C
    // =========================================================================

    II2c &bus;

    uint8_t address;

    bool initialized = false;

    // =========================================================================
    // Calibration
    // =========================================================================

    uint16_t dig_T1 = 0;
    int16_t dig_T2 = 0;
    int16_t dig_T3 = 0;

    uint16_t dig_P1 = 0;
    int16_t dig_P2 = 0;
    int16_t dig_P3 = 0;
    int16_t dig_P4 = 0;
    int16_t dig_P5 = 0;
    int16_t dig_P6 = 0;
    int16_t dig_P7 = 0;
    int16_t dig_P8 = 0;
    int16_t dig_P9 = 0;

    int32_t temperatureFine = 0;

private:
    // =========================================================================
    // Register operations
    // =========================================================================

    bool readRegister(
        uint8_t reg,
        uint8_t &value);

    bool writeRegister(
        uint8_t reg,
        uint8_t value);

    bool readRegisters(
        uint8_t reg,
        uint8_t *data,
        uint32_t length);

    // =========================================================================
    // Initialization
    // =========================================================================

    bool readCalibration();

    // =========================================================================
    // Measurement
    // =========================================================================

    bool triggerMeasurement();

    bool waitForMeasurement();

    bool readRaw(
        int32_t &temperature,
        int32_t &pressure);

    // =========================================================================
    // Compensation
    // =========================================================================

    float compensateTemperature(
        int32_t rawTemperature);

    float compensatePressure(
        int32_t rawPressure);
};