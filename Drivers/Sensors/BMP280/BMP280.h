#pragma once

#include <cstdint>

#include "II2c.h"

class BMP280
{
public:
    struct Measurements
    {
        float temperatureC;
        float pressurePa;
        float pressureHpa;
    };

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
    // -------------------------------------------------------------------------
    // Registers
    // -------------------------------------------------------------------------

    static constexpr uint8_t REG_ID = 0xD0;
    static constexpr uint8_t REG_RESET = 0xE0;
    static constexpr uint8_t REG_STATUS = 0xF3;
    static constexpr uint8_t REG_CTRL_MEAS = 0xF4;
    static constexpr uint8_t REG_CONFIG = 0xF5;
    static constexpr uint8_t REG_DATA = 0xF7;
    static constexpr uint8_t REG_CALIB = 0x88;

    // -------------------------------------------------------------------------
    // Device
    // -------------------------------------------------------------------------

    static constexpr uint8_t DEVICE_ID = 0x58;

    // -------------------------------------------------------------------------
    // I2C
    // -------------------------------------------------------------------------

    II2c &bus;
    uint8_t address;

    bool initialized = false;

    // -------------------------------------------------------------------------
    // Calibration coefficients
    // -------------------------------------------------------------------------

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

    // -------------------------------------------------------------------------
    // Temperature compensation state
    // -------------------------------------------------------------------------

    int32_t temperatureFine = 0;

    // -------------------------------------------------------------------------
    // Internal functions
    // -------------------------------------------------------------------------

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

    bool readCalibration();

    bool readRaw(
        int32_t &temperature,
        int32_t &pressure);

    float compensateTemperature(
        int32_t rawTemperature);

    float compensatePressure(
        int32_t rawPressure);
};