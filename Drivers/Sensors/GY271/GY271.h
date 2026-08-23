#pragma once

#include <cstdint>

#include "II2c.h"

class GY271
{
public:
    struct MagneticField
    {
        float x;
        float y;
        float z;
    };

    explicit GY271(
        II2c &bus,
        uint8_t address = 0x1E);

    bool initialize();

    bool readRaw(
        int16_t &x,
        int16_t &y,
        int16_t &z);

    bool read(
        MagneticField &field);

    bool isConnected();

private:
    // =========================================================================
    // HMC5883L Register Map
    // =========================================================================

    static constexpr uint8_t REG_CONFIG_A = 0x00;
    static constexpr uint8_t REG_CONFIG_B = 0x01;
    static constexpr uint8_t REG_MODE = 0x02;

    static constexpr uint8_t REG_DATA_X_MSB = 0x03;
    static constexpr uint8_t REG_DATA_X_LSB = 0x04;

    static constexpr uint8_t REG_DATA_Z_MSB = 0x05;
    static constexpr uint8_t REG_DATA_Z_LSB = 0x06;

    static constexpr uint8_t REG_DATA_Y_MSB = 0x07;
    static constexpr uint8_t REG_DATA_Y_LSB = 0x08;

    static constexpr uint8_t REG_STATUS = 0x09;

    static constexpr uint8_t REG_ID_A = 0x0A;
    static constexpr uint8_t REG_ID_B = 0x0B;
    static constexpr uint8_t REG_ID_C = 0x0C;

    // =========================================================================
    // CONFIG A
    // =========================================================================

    // 8 samples averaged
    // 15 Hz output rate
    // Normal measurement
    static constexpr uint8_t CONFIG_A_8_AVG_15HZ =
        0x70;

    // =========================================================================
    // CONFIG B
    // =========================================================================

    // Gain = 1.3 Ga
    //
    // HMC5883L:
    // GN = 001
    // Recommended for normal operation.
    //
    // Sensitivity:
    // 1090 LSB/Gauss
    //
    static constexpr uint8_t CONFIG_B_GAIN_1_3GA =
        0x20;

    // =========================================================================
    // MODE
    // =========================================================================

    // Continuous measurement mode
    static constexpr uint8_t MODE_CONTINUOUS =
        0x00;

    // =========================================================================
    // STATUS
    // =========================================================================

    static constexpr uint8_t STATUS_RDY = 0x01;
    static constexpr uint8_t STATUS_LOCK = 0x02;

    // =========================================================================
    // Device ID
    // =========================================================================

    static constexpr uint8_t ID_A = 0x48; // 'H'
    static constexpr uint8_t ID_B = 0x34; // '4'
    static constexpr uint8_t ID_C = 0x33; // '3'

    // =========================================================================
    // Scale
    // =========================================================================

    // CONFIG_B = 0x20
    // Gain = 1.3 Ga
    //
    // Sensitivity = 1090 LSB/Gauss
    //
    // Therefore:
    //
    // Gauss per LSB = 1 / 1090
    //
    static constexpr float GAUSS_PER_LSB =
        1.0f / 1090.0f;

    // =========================================================================
    // Hardware
    // =========================================================================

    II2c &bus;

    uint8_t address;

    bool initialized = false;

    float scale = GAUSS_PER_LSB;

    // =========================================================================
    // Register access
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
};