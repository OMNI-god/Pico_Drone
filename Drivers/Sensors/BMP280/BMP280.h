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
        float altitudeM;
        bool valid;
    };

    explicit BMP280(
        II2c &bus,
        uint8_t address = 0x76,
        float seaLevelHpa = 1013.25f);

    bool initialize();

    bool read(
        Measurements &measurements);

    bool readTemperature(
        float &temperatureC);

    bool readPressure(
        float &pressurePa);

    bool isConnected();

    bool test(
        uint32_t timeoutMs = 3000);

    void setSeaLevelPressure(
        float hpa);

    bool isValid() const;

private:
    // ========================================================================
    // Registers
    // ========================================================================

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

    static constexpr uint8_t REG_CALIB =
        0x88;

    // ========================================================================
    // Device
    // ========================================================================

    static constexpr uint8_t CHIP_ID =
        0x58;

    static constexpr uint8_t RESET_COMMAND =
        0xB6;

    // ========================================================================
    // Python-equivalent configuration
    // ========================================================================

    /*
     * Python:
     *
     * self._write8(self.REG_CTRL_MEAS, 0x27)
     *
     * osrs_t = 001 = x1
     * osrs_p = 001 = x1
     * mode   = 11  = normal
     *
     * 0x27
     */

    static constexpr uint8_t CTRL_MEAS_NORMAL =
        0x27;

    /*
     * Python:
     *
     * self._write8(self.REG_CONFIG, 0xA0)
     *
     * t_sb    = 101 = 1000 ms
     * filter  = 100 = x16
     *
     * 0xA0
     */

    static constexpr uint8_t CONFIG_VALUE =
        0xA0;

    // ========================================================================
    // Raw measurement
    // ========================================================================

    static constexpr int32_t ADC_INVALID =
        0x80000;

    // ========================================================================
    // I2C
    // ========================================================================

    II2c &bus;

    uint8_t address;

    float seaLevelHpa;

    bool initialized = false;

    bool valid = false;

    // ========================================================================
    // Calibration
    // ========================================================================

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

    float temperatureFine = 0.0f;

    // ========================================================================
    // Low-level register functions
    // ========================================================================

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

    // ========================================================================
    // Initialization
    // ========================================================================

    bool readCalibration();

    // ========================================================================
    // Measurement
    // ========================================================================

    bool readRaw(
        int32_t &temperature,
        int32_t &pressure);

    // ========================================================================
    // Compensation
    // ========================================================================

    float compensateTemperature(
        int32_t rawTemperature);

    float compensatePressure(
        int32_t rawPressure);

    // ========================================================================
    // Altitude
    // ========================================================================

    float altitudeFromPressure(
        float pressureHpa);
};