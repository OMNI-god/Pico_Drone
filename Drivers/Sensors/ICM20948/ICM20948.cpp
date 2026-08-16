#include "ICM20948.h"

#include "pico/stdlib.h"

// =============================================================================
// Constructor
// =============================================================================

ICM20948::ICM20948(
    II2c &bus,
    uint8_t address)
    : bus(bus),
      address(address)
{
}

// =============================================================================
// Initialization
// =============================================================================

bool ICM20948::initialize()
{
    initialized = false;

    // -------------------------------------------------------------------------
    // Reset internal bank tracking.
    //
    // After a hardware reset the ICM-20948 starts in bank 0.
    // -------------------------------------------------------------------------

    currentBank = 0xFF;

    // -------------------------------------------------------------------------
    // Select register bank 0
    // -------------------------------------------------------------------------

    if (!selectBank(0))
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Verify device identity
    // -------------------------------------------------------------------------

    uint8_t deviceId = 0;

    if (!readRegister(
            REG_WHO_AM_I,
            deviceId))
    {
        return false;
    }

    if (deviceId != DEVICE_ID)
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Reset device
    // -------------------------------------------------------------------------

    if (!writeRegister(
            REG_PWR_MGMT_1,
            0x80))
    {
        return false;
    }

    // Allow reset to complete.
    sleep_ms(100);

    // -------------------------------------------------------------------------
    // Wake device
    //
    // CLKSEL = 001
    // Auto selects best available clock.
    // -------------------------------------------------------------------------

    if (!writeRegister(
            REG_PWR_MGMT_1,
            0x01))
    {
        return false;
    }

    sleep_ms(10);

    // -------------------------------------------------------------------------
    // Enable accelerometer and gyroscope
    //
    // 0x00 = all sensors enabled
    // -------------------------------------------------------------------------

    if (!writeRegister(
            REG_PWR_MGMT_2,
            0x00))
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Select register bank 2
    // -------------------------------------------------------------------------

    if (!selectBank(2))
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Configure Gyroscope
    //
    // GYRO_CONFIG_1
    //
    // FS_SEL = 0
    // ±250 degrees/sec
    //
    // DLPF disabled.
    // -------------------------------------------------------------------------

    if (!writeRegister(
            REG_GYRO_CONFIG_1,
            0x00))
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Configure Accelerometer
    //
    // ACCEL_CONFIG
    //
    // FS_SEL = 0
    // ±2 g
    //
    // DLPF disabled.
    // -------------------------------------------------------------------------

    if (!writeRegister(
            REG_ACCEL_CONFIG,
            0x00))
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Return to register bank 0
    // -------------------------------------------------------------------------

    if (!selectBank(0))
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Initialization complete
    // -------------------------------------------------------------------------

    initialized = true;

    return true;
}

// =============================================================================
// Read Accelerometer
// =============================================================================

bool ICM20948::readAcceleration(
    Acceleration &acceleration)
{
    if (!initialized)
    {
        return false;
    }

    uint8_t buffer[6];

    if (!readRegisters(
            REG_ACCEL_XOUT_H,
            buffer,
            sizeof(buffer)))
    {
        return false;
    }

    const int16_t x =
        static_cast<int16_t>(
            (static_cast<uint16_t>(buffer[0]) << 8) |
            buffer[1]);

    const int16_t y =
        static_cast<int16_t>(
            (static_cast<uint16_t>(buffer[2]) << 8) |
            buffer[3]);

    const int16_t z =
        static_cast<int16_t>(
            (static_cast<uint16_t>(buffer[4]) << 8) |
            buffer[5]);

    acceleration.x =
        x * accelScale;

    acceleration.y =
        y * accelScale;

    acceleration.z =
        z * accelScale;

    return true;
}

// =============================================================================
// Read Gyroscope
// =============================================================================

bool ICM20948::readGyroscope(
    Gyroscope &gyroscope)
{
    if (!initialized)
    {
        return false;
    }

    uint8_t buffer[6];

    if (!readRegisters(
            REG_GYRO_XOUT_H,
            buffer,
            sizeof(buffer)))
    {
        return false;
    }

    const int16_t x =
        static_cast<int16_t>(
            (static_cast<uint16_t>(buffer[0]) << 8) |
            buffer[1]);

    const int16_t y =
        static_cast<int16_t>(
            (static_cast<uint16_t>(buffer[2]) << 8) |
            buffer[3]);

    const int16_t z =
        static_cast<int16_t>(
            (static_cast<uint16_t>(buffer[4]) << 8) |
            buffer[5]);

    gyroscope.x =
        x * gyroScale;

    gyroscope.y =
        y * gyroScale;

    gyroscope.z =
        z * gyroScale;

    return true;
}

// =============================================================================
// Read Accelerometer + Gyroscope
// =============================================================================

bool ICM20948::read(
    SensorData &data)
{
    if (!initialized)
    {
        return false;
    }

    // Accelerometer = 6 bytes
    // Gyroscope     = 6 bytes
    // Total         = 12 bytes

    uint8_t buffer[12];

    if (!readRegisters(
            REG_ACCEL_XOUT_H,
            buffer,
            sizeof(buffer)))
    {
        return false;
    }

    const int16_t ax =
        static_cast<int16_t>(
            (static_cast<uint16_t>(buffer[0]) << 8) |
            buffer[1]);

    const int16_t ay =
        static_cast<int16_t>(
            (static_cast<uint16_t>(buffer[2]) << 8) |
            buffer[3]);

    const int16_t az =
        static_cast<int16_t>(
            (static_cast<uint16_t>(buffer[4]) << 8) |
            buffer[5]);

    const int16_t gx =
        static_cast<int16_t>(
            (static_cast<uint16_t>(buffer[6]) << 8) |
            buffer[7]);

    const int16_t gy =
        static_cast<int16_t>(
            (static_cast<uint16_t>(buffer[8]) << 8) |
            buffer[9]);

    const int16_t gz =
        static_cast<int16_t>(
            (static_cast<uint16_t>(buffer[10]) << 8) |
            buffer[11]);

    data.acceleration =
        {
            ax * accelScale,
            ay * accelScale,
            az * accelScale};

    data.gyroscope =
        {
            gx * gyroScale,
            gy * gyroScale,
            gz * gyroScale};

    return true;
}

// =============================================================================
// Check Device Connection
// =============================================================================

bool ICM20948::isConnected()
{
    if (!selectBank(0))
    {
        return false;
    }

    uint8_t deviceId = 0;

    if (!readRegister(
            REG_WHO_AM_I,
            deviceId))
    {
        return false;
    }

    return deviceId == DEVICE_ID;
}

// =============================================================================
// Select Register Bank
// =============================================================================

bool ICM20948::selectBank(
    uint8_t bank)
{
    bank &= 0x03;

    // Avoid unnecessary I2C transactions.
    if (currentBank == bank)
    {
        return true;
    }

    const uint8_t value =
        static_cast<uint8_t>(
            bank << 4);

    if (!writeRegister(
            REG_BANK_SEL,
            value))
    {
        return false;
    }

    currentBank = bank;

    return true;
}

// =============================================================================
// Read Single Register
// =============================================================================

bool ICM20948::readRegister(
    uint8_t reg,
    uint8_t &value)
{
    return readRegisters(
        reg,
        &value,
        1);
}

// =============================================================================
// Write Single Register
// =============================================================================

bool ICM20948::writeRegister(
    uint8_t reg,
    uint8_t value)
{
    const uint8_t buffer[2] =
        {
            reg,
            value};

    return bus.write(
               address,
               buffer,
               sizeof(buffer)) ==
           static_cast<int>(sizeof(buffer));
}

// =============================================================================
// Read Multiple Registers
// =============================================================================

bool ICM20948::readRegisters(
    uint8_t reg,
    uint8_t *data,
    uint32_t length)
{
    if (data == nullptr)
    {
        return false;
    }

    if (length == 0)
    {
        return false;
    }

    return bus.writeRead(
               address,
               &reg,
               1,
               data,
               length) ==
           static_cast<int>(length);
}