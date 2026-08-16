#pragma once

#include <cstdint>

#include "II2c.h"

class ICM20948
{
public:
    // =========================================================================
    // Data Types
    // =========================================================================

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

    // =========================================================================
    // Constructor
    // =========================================================================

    explicit ICM20948(
        II2c &bus,
        uint8_t address = 0x68);

    // =========================================================================
    // Public API
    // =========================================================================

    bool initialize();

    bool readAcceleration(
        Acceleration &acceleration);

    bool readGyroscope(
        Gyroscope &gyroscope);

    bool read(
        SensorData &data);

    bool isConnected();

private:
    // =========================================================================
    // ICM-20948 Register Map - Bank 0
    // =========================================================================

    static constexpr uint8_t REG_WHO_AM_I =
        0x00;

    static constexpr uint8_t REG_USER_CTRL =
        0x03;

    static constexpr uint8_t REG_PWR_MGMT_1 =
        0x06;

    static constexpr uint8_t REG_PWR_MGMT_2 =
        0x07;

    static constexpr uint8_t REG_ACCEL_XOUT_H =
        0x2D;

    static constexpr uint8_t REG_GYRO_XOUT_H =
        0x33;

    static constexpr uint8_t REG_BANK_SEL =
        0x7F;

    // =========================================================================
    // ICM-20948 Register Map - Bank 2
    // =========================================================================

    static constexpr uint8_t REG_GYRO_CONFIG_1 =
        0x01;

    static constexpr uint8_t REG_ACCEL_CONFIG =
        0x14;

    // =========================================================================
    // Device Information
    // =========================================================================

    static constexpr uint8_t DEVICE_ID =
        0xEA;

    // =========================================================================
    // Configuration
    // =========================================================================

    // Accelerometer:
    // ±2 g
    // 16384 LSB/g
    static constexpr float ACCEL_SCALE =
        1.0f / 16384.0f;

    // Gyroscope:
    // ±250 dps
    // 131 LSB/(degrees/sec)
    static constexpr float GYRO_SCALE =
        1.0f / 131.0f;

    // =========================================================================
    // State
    // =========================================================================

    II2c &bus;

    uint8_t address;

    bool initialized = false;

    // 0xFF means unknown bank.
    uint8_t currentBank = 0xFF;

    float accelScale = ACCEL_SCALE;

    float gyroScale = GYRO_SCALE;

    // =========================================================================
    // Register Access
    // =========================================================================

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
};