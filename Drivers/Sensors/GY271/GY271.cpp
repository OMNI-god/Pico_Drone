#include "GY271.h"

#include "pico/stdlib.h"

#include <cstdio>

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

    printf("GY271: initializing...\n");

    // -------------------------------------------------------------------------
    // Check I2C communication
    // -------------------------------------------------------------------------

    if (!isConnected())
    {
        printf("GY271: connection failed\n");
        return false;
    }

    printf("GY271: connection OK\n");

    // -------------------------------------------------------------------------
    // Read initial register state
    // -------------------------------------------------------------------------

    uint8_t control1 = 0;

    if (readRegister(
            REG_CONTROL_1,
            control1))
    {
        printf(
            "GY271: initial CONTROL_1 = 0x%02X\n",
            control1);
    }

    // -------------------------------------------------------------------------
    // Soft reset
    // -------------------------------------------------------------------------

    printf("GY271: resetting...\n");

    if (!writeRegister(
            REG_CONTROL_2,
            CONTROL_2_SOFT_RESET))
    {
        printf("GY271: reset failed\n");
        return false;
    }

    sleep_ms(10);

    // -------------------------------------------------------------------------
    // SET/RESET period
    // -------------------------------------------------------------------------

    if (!writeRegister(
            REG_SET_RESET,
            0x01))
    {
        printf("GY271: SET_RESET failed\n");
        return false;
    }

    // -------------------------------------------------------------------------
    // Configure continuous measurement
    // -------------------------------------------------------------------------

    printf(
        "GY271: writing CONTROL_1 = 0x%02X\n",
        CONTROL_1_CONTINUOUS_200HZ);

    if (!writeRegister(
            REG_CONTROL_1,
            CONTROL_1_CONTINUOUS_200HZ))
    {
        printf("GY271: CONTROL_1 write failed\n");
        return false;
    }

    sleep_ms(10);

    // -------------------------------------------------------------------------
    // Read CONTROL_1 back
    // -------------------------------------------------------------------------

    control1 = 0;

    if (!readRegister(
            REG_CONTROL_1,
            control1))
    {
        printf("GY271: CONTROL_1 read failed\n");
        return false;
    }

    printf(
        "GY271: CONTROL_1 = 0x%02X\n",
        control1);

    // -------------------------------------------------------------------------
    // Read STATUS
    // -------------------------------------------------------------------------

    uint8_t status = 0;

    if (!readRegister(
            REG_STATUS,
            status))
    {
        printf("GY271: STATUS read failed\n");
        return false;
    }

    printf(
        "GY271: STATUS = 0x%02X\n",
        status);

    // -------------------------------------------------------------------------
    // Do NOT require DRDY during initialization.
    //
    // The sensor may need some time before the first measurement.
    // -------------------------------------------------------------------------

    initialized = true;

    printf("GY271: initialized\n");

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
    // Read status
    // -------------------------------------------------------------------------

    uint8_t status = 0;

    if (!readRegister(
            REG_STATUS,
            status))
    {
        printf("GY271: status read failed\n");
        return false;
    }

    // -------------------------------------------------------------------------
    // Data ready?
    // -------------------------------------------------------------------------

    if (!(status & STATUS_DRDY))
    {
        // This is not necessarily an error.
        //
        // The sensor may simply not have produced a new sample yet.
        return false;
    }

    // -------------------------------------------------------------------------
    // Magnetic overflow
    // -------------------------------------------------------------------------

    if (status & STATUS_OVL)
    {
        printf(
            "GY271: magnetic overflow, STATUS=0x%02X\n",
            status);

        // Do not use saturated data.
        return false;
    }

    // -------------------------------------------------------------------------
    // Read X/Y/Z
    // -------------------------------------------------------------------------

    uint8_t data[6] = {};

    if (!readRegisters(
            REG_X_LSB,
            data,
            sizeof(data)))
    {
        printf("GY271: magnetic data read failed\n");
        return false;
    }

    // QMC5883L:
    //
    // X LSB
    // X MSB
    // Y LSB
    // Y MSB
    // Z LSB
    // Z MSB

    x = static_cast<int16_t>(
        static_cast<uint16_t>(data[1]) << 8 |
        data[0]);

    y = static_cast<int16_t>(
        static_cast<uint16_t>(data[3]) << 8 |
        data[2]);

    z = static_cast<int16_t>(
        static_cast<uint16_t>(data[5]) << 8 |
        data[4]);

    return true;
}

// =============================================================================
// Read Magnetic Field
// =============================================================================

bool GY271::read(
    MagneticField &field)
{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    if (!readRaw(
            x,
            y,
            z))
    {
        return false;
    }

    field.x = static_cast<float>(x) * scale;
    field.y = static_cast<float>(y) * scale;
    field.z = static_cast<float>(z) * scale;

    return true;
}

// =============================================================================
// Check Device Connection
// =============================================================================

bool GY271::isConnected()
{
    uint8_t value = 0;

    // -------------------------------------------------------------------------
    // There is no reliable WHO_AM_I register on the QMC5883L.
    //
    // Therefore perform an actual I2C transaction.
    //
    // STATUS is a better register to use than CONTROL_1 because we don't
    // depend on a particular power/configuration state.
    // -------------------------------------------------------------------------

    return readRegister(
        REG_STATUS,
        value);
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

    const int result =
        bus.write(
            address,
            data,
            sizeof(data));

    return result ==
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

    const int result =
        bus.writeRead(
            address,
            &reg,
            1,
            data,
            length);

    return result ==
           static_cast<int>(length);
}