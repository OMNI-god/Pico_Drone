#include "ADXL345.h"

ADXL345::ADXL345(
    II2c &bus,
    uint8_t address)
    : bus(bus),
      address(address)
{
}

bool ADXL345::initialize(Range range)
{
    uint8_t deviceId = 0;

    // Check device ID
    if (!readRegister(
            REG_DEVID,
            deviceId))
    {
        return false;
    }

    if (deviceId != DEVICE_ID)
    {
        return false;
    }

    // Configure measurement range
    uint8_t rangeBits =
        static_cast<uint8_t>(range);

    uint8_t dataFormat =
        0x08 | rangeBits;

    if (!writeRegister(
            REG_DATA_FORMAT,
            dataFormat))
    {
        return false;
    }

    // 100 Hz output data rate
    if (!writeRegister(
            REG_BW_RATE,
            0x0A))
    {
        return false;
    }

    // Enable measurement mode
    if (!writeRegister(
            REG_POWER_CTL,
            0x08))
    {
        return false;
    }

    // Set scale
    switch (range)
    {
    case Range::RANGE_2G:
        scaleG = 0.0039f;
        break;

    case Range::RANGE_4G:
        scaleG = 0.0078f;
        break;

    case Range::RANGE_8G:
        scaleG = 0.0156f;
        break;

    case Range::RANGE_16G:
        scaleG = 0.0312f;
        break;
    }

    initialized = true;

    return true;
}

bool ADXL345::readRaw(
    int16_t &x,
    int16_t &y,
    int16_t &z)
{
    if (!initialized)
    {
        return false;
    }

    uint8_t data[6];

    if (!readRegisters(
            REG_DATAX0,
            data,
            sizeof(data)))
    {
        return false;
    }

    x = static_cast<int16_t>(
        (static_cast<uint16_t>(data[1]) << 8) |
        data[0]);

    y = static_cast<int16_t>(
        (static_cast<uint16_t>(data[3]) << 8) |
        data[2]);

    z = static_cast<int16_t>(
        (static_cast<uint16_t>(data[5]) << 8) |
        data[4]);

    return true;
}

bool ADXL345::readAcceleration(
    Acceleration &acceleration)
{
    int16_t x;
    int16_t y;
    int16_t z;

    if (!readRaw(
            x,
            y,
            z))
    {
        return false;
    }

    acceleration.x =
        static_cast<float>(x) * scaleG;

    acceleration.y =
        static_cast<float>(y) * scaleG;

    acceleration.z =
        static_cast<float>(z) * scaleG;

    return true;
}

bool ADXL345::isConnected()
{
    uint8_t id = 0;

    if (!readRegister(
            REG_DEVID,
            id))
    {
        return false;
    }

    return id == DEVICE_ID;
}

bool ADXL345::readRegister(
    uint8_t reg,
    uint8_t &value)
{
    return readRegisters(
        reg,
        &value,
        1);
}

bool ADXL345::writeRegister(
    uint8_t reg,
    uint8_t value)
{
    uint8_t data[2] =
        {
            reg,
            value};

    return bus.write(
               address,
               data,
               sizeof(data),
               false) == 2;
}

bool ADXL345::readRegisters(
    uint8_t reg,
    uint8_t *data,
    uint32_t length)
{
    /*
     * ADXL345 expects:
     *
     * START
     * ADDRESS + WRITE
     * REGISTER
     * REPEATED START
     * ADDRESS + READ
     * DATA...
     * STOP
     *
     * Therefore the first write must use nostop=true.
     */

    if (bus.write(
            address,
            &reg,
            1,
            true) != 1)
    {
        return false;
    }

    return bus.read(
               address,
               data,
               length,
               false) == static_cast<int>(length);
}