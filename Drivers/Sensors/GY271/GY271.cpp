#include "GY271.h"
#include "pico/stdlib.h"

// =============================================================================
// Constructor
// =============================================================================

GY271::GY271(
    II2c &bus,
    uint8_t address)
    : bus(bus),
      address(address)
{
}

// =============================================================================
// Initialization
// =============================================================================

bool GY271::initialize()
{
    initialized = false;

    // -------------------------------------------------------------------------
    // Verify I2C communication before configuration.
    // -------------------------------------------------------------------------

    if (!isConnected())
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Soft reset
    // -------------------------------------------------------------------------

    if (!writeRegister(
            REG_CONTROL_2,
            CONTROL_2_SOFT_RESET))
    {
        return false;
    }

    // Give the device time to reset.
    sleep_ms(10);

    // -------------------------------------------------------------------------
    // Set/reset period.
    //
    // This register controls the set/reset period of the magnetometer.
    // -------------------------------------------------------------------------

    if (!writeRegister(
            REG_SET_RESET,
            0x01))
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Configure magnetometer.
    //
    // OSR  = 512
    // RNG  = ±8 G
    // ODR  = 200 Hz
    // MODE = Continuous
    // -------------------------------------------------------------------------

    if (!writeRegister(
            REG_CONTROL_1,
            CONTROL_1_CONTINUOUS_200HZ))
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Driver is now initialized.
    // -------------------------------------------------------------------------

    initialized = true;

    return true;
}

// =============================================================================
// Read Raw Magnetic Field
// =============================================================================

bool GY271::readRaw(
    int16_t &x,
    int16_t &y,
    int16_t &z)
{
    if (!initialized)
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Check status.
    // -------------------------------------------------------------------------

    uint8_t status = 0;

    if (!readRegister(
            REG_STATUS,
            status))
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Data not ready.
    // -------------------------------------------------------------------------

    if ((status & STATUS_DRDY) == 0)
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Magnetic sensor overflow.
    //
    // The reading is not reliable.
    // -------------------------------------------------------------------------

    if ((status & STATUS_OVL) != 0)
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Read X/Y/Z in a single I2C transaction.
    //
    // Register sequence:
    //
    // X_LSB
    // X_MSB
    // Y_LSB
    // Y_MSB
    // Z_LSB
    // Z_MSB
    // -------------------------------------------------------------------------

    uint8_t data[6];

    if (!readRegisters(
            REG_X_LSB,
            data,
            sizeof(data)))
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Convert little-endian sensor data to signed 16-bit values.
    // -------------------------------------------------------------------------

    x =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[0]) |
            (static_cast<uint16_t>(data[1]) << 8));

    y =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[2]) |
            (static_cast<uint16_t>(data[3]) << 8));

    z =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[4]) |
            (static_cast<uint16_t>(data[5]) << 8));

    return true;
}

// =============================================================================
// Read Magnetic Field
// =============================================================================

bool GY271::read(
    MagneticField &field)
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

    field.x =
        x * scale;

    field.y =
        y * scale;

    field.z =
        z * scale;

    return true;
}

// =============================================================================
// Check Device Connection
// =============================================================================

bool GY271::isConnected()
{
    uint8_t control = 0;

    // QMC5883L does not expose a conventional
    // WHO_AM_I register.
    //
    // Therefore, successful communication with
    // a valid register is used as the connection test.

    return readRegister(
        REG_CONTROL_1,
        control);
}

// =============================================================================
// Read Single Register
// =============================================================================

bool GY271::readRegister(
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

bool GY271::writeRegister(
    uint8_t reg,
    uint8_t value)
{
    const uint8_t data[2] =
        {
            reg,
            value};

    return bus.write(
               address,
               data,
               sizeof(data)) ==
           static_cast<int>(sizeof(data));
}

// =============================================================================
// Read Multiple Registers
// =============================================================================

bool GY271::readRegisters(
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