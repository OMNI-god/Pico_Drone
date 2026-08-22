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
    // QMC5883L Register Map
    // =========================================================================

    static constexpr uint8_t REG_X_LSB = 0x00;
    static constexpr uint8_t REG_X_MSB = 0x01;

    static constexpr uint8_t REG_Y_LSB = 0x02;
    static constexpr uint8_t REG_Y_MSB = 0x03;

    static constexpr uint8_t REG_Z_LSB = 0x04;
    static constexpr uint8_t REG_Z_MSB = 0x05;

    static constexpr uint8_t REG_STATUS = 0x06;

    static constexpr uint8_t REG_CONTROL_1 = 0x09;
    static constexpr uint8_t REG_CONTROL_2 = 0x0A;
    static constexpr uint8_t REG_SET_RESET = 0x0B;

    // =========================================================================
    // STATUS
    // =========================================================================

    static constexpr uint8_t STATUS_DRDY = 0x01;
    static constexpr uint8_t STATUS_OVL = 0x02;
    static constexpr uint8_t STATUS_DOR = 0x04;

    // =========================================================================
    // CONTROL 1
    // =========================================================================

    static constexpr uint8_t CONTROL_1_CONTINUOUS_200HZ = 0x1D;

    // =========================================================================
    // CONTROL 2
    // =========================================================================

    static constexpr uint8_t CONTROL_2_SOFT_RESET = 0x80;

    // =========================================================================
    // Scale
    // =========================================================================

    static constexpr float GAUSS_PER_LSB = 0.0003f;

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