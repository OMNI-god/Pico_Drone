#pragma once

#include <cstdint>

#include "II2c.h"

class ICM20948
{
public:
    struct Acceleration
    {
        float x;
        float y;
        float z;
    };

    struct Gyroscope
    {
        float x;
        float y;
        float z;
    };

    struct SensorData
    {
        Acceleration acceleration;
        Gyroscope gyroscope;
    };

    explicit ICM20948(
        II2c &bus,
        uint8_t address = 0x69);

    bool initialize();

    bool readAcceleration(
        Acceleration &acceleration);

    bool readGyroscope(
        Gyroscope &gyroscope);

    bool read(
        SensorData &data);

    bool isConnected();

    uint8_t getAddress() const;

private:
    // =====================================================================
    // I2C ADDRESSES
    // =====================================================================

    static constexpr uint8_t ADDRESS_68 = 0x68;
    static constexpr uint8_t ADDRESS_69 = 0x69;

    // =====================================================================
    // BANK 0 REGISTERS
    // =====================================================================

    static constexpr uint8_t REG_WHO_AM_I = 0x00;
    static constexpr uint8_t REG_USER_CTRL = 0x03;
    static constexpr uint8_t REG_PWR_MGMT_1 = 0x06;
    static constexpr uint8_t REG_PWR_MGMT_2 = 0x07;

    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x2D;
    static constexpr uint8_t REG_GYRO_XOUT_H = 0x33;

    static constexpr uint8_t REG_BANK_SEL = 0x7F;

    // =====================================================================
    // BANK 2 REGISTERS
    // =====================================================================

    static constexpr uint8_t REG_GYRO_CONFIG_1 = 0x01;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x14;

    // =====================================================================
    // DEVICE ID
    // =====================================================================

    static constexpr uint8_t DEVICE_ID = 0xEA;

    // =====================================================================
    // SENSOR SCALE
    // =====================================================================

    // Accelerometer ±2g
    // 16384 LSB/g

    static constexpr float ACCEL_SCALE =
        1.0f / 16384.0f;

    // Gyroscope ±250 DPS
    // 131 LSB/DPS

    static constexpr float GYRO_SCALE =
        1.0f / 131.0f;

    // =====================================================================
    // STATE
    // =====================================================================

    II2c &bus;

    uint8_t address;

    bool initialized;

    uint8_t currentBank;

    float accelScale;

    float gyroScale;

    // =====================================================================
    // LOW LEVEL I2C
    // =====================================================================

    bool selectBank(
        uint8_t bank);

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

    // =====================================================================
    // DEVICE DETECTION
    // =====================================================================

    bool detectDevice();

    bool readWhoAmI(
        uint8_t &value);

    // =====================================================================
    // DATA CONVERSION
    // =====================================================================

    static int16_t makeInt16(
        uint8_t high,
        uint8_t low);
};