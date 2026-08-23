#pragma once

#include <cstdint>

#include "II2c.h"

class ADXL345
{
public:
    struct Acceleration
    {
        float x;
        float y;
        float z;
    };

    explicit ADXL345(
        II2c &bus,
        uint8_t address = 0x53);

    bool initialize();

    bool readAcceleration(
        Acceleration &acceleration);

    bool isConnected();

    uint8_t getAddress() const;

private:
    // ============================================================
    // I2C ADDRESSES
    // ============================================================

    static constexpr uint8_t ADDRESS_LOW = 0x53;
    static constexpr uint8_t ADDRESS_HIGH = 0x1D;

    // ============================================================
    // REGISTERS
    // ============================================================

    static constexpr uint8_t REG_DEVID = 0x00;

    static constexpr uint8_t REG_POWER_CTL = 0x2D;

    static constexpr uint8_t REG_DATA_FORMAT = 0x31;

    static constexpr uint8_t REG_DATAX0 = 0x32;

    // ============================================================
    // DEVICE ID
    // ============================================================

    static constexpr uint8_t DEVICE_ID = 0xE5;

    // ============================================================
    // SCALE
    // ============================================================

    // Full resolution mode.
    //
    // Approximately 3.9 mg/LSB.
    //
    static constexpr float ACCEL_SCALE = 0.0039f;

    // ============================================================
    // STATE
    // ============================================================

    II2c &bus;

    uint8_t address;

    bool initialized;

    // ============================================================
    // LOW LEVEL ACCESS
    // ============================================================

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

    // ============================================================
    // DEVICE DETECTION
    // ============================================================

    bool detectDevice();

    bool readDeviceId(
        uint8_t &deviceId);

    // ============================================================
    // CONVERSION
    // ============================================================

    static int16_t makeInt16(
        uint8_t low,
        uint8_t high);
};