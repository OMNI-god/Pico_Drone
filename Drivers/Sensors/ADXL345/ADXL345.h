#pragma once

#include <cstdint>

#include "II2c.h"

class ADXL345
{
public:
    enum class Range : uint8_t
    {
        RANGE_2G = 0,
        RANGE_4G = 1,
        RANGE_8G = 2,
        RANGE_16G = 3
    };

    struct Acceleration
    {
        float x;
        float y;
        float z;
    };

    explicit ADXL345(
        II2c &bus,
        uint8_t address = 0x53);

    bool initialize(
        Range range = Range::RANGE_4G);

    bool readRaw(
        int16_t &x,
        int16_t &y,
        int16_t &z);

    bool readAcceleration(
        Acceleration &acceleration);

    bool isConnected();

private:
    static constexpr uint8_t REG_DEVID = 0x00;
    static constexpr uint8_t REG_BW_RATE = 0x2C;
    static constexpr uint8_t REG_POWER_CTL = 0x2D;
    static constexpr uint8_t REG_DATA_FORMAT = 0x31;
    static constexpr uint8_t REG_DATAX0 = 0x32;

    static constexpr uint8_t DEVICE_ID = 0xE5;

    II2c &bus;

    uint8_t address;

    float scaleG = 0.0039f;

    bool initialized = false;

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