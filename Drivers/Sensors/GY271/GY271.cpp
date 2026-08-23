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

    printf("\n");
    printf("==============================\n");
    printf(" GY271 HMC5883L INITIALIZATION\n");
    printf("==============================\n");

    printf(
        "GY271: checking address 0x%02X...\n",
        address);

    // =========================================================================
    // Check connection
    // =========================================================================

    if (!isConnected())
    {
        printf(
            "GY271: device not found at 0x%02X\n",
            address);

        return false;
    }

    printf(
        "GY271: device found at 0x%02X\n",
        address);

    // =========================================================================
    // Read device ID
    // =========================================================================

    uint8_t idA = 0;
    uint8_t idB = 0;
    uint8_t idC = 0;

    if (!readRegister(
            REG_ID_A,
            idA))
    {
        printf("GY271: failed to read ID A\n");
        return false;
    }

    if (!readRegister(
            REG_ID_B,
            idB))
    {
        printf("GY271: failed to read ID B\n");
        return false;
    }

    if (!readRegister(
            REG_ID_C,
            idC))
    {
        printf("GY271: failed to read ID C\n");
        return false;
    }

    printf(
        "GY271: ID = 0x%02X 0x%02X 0x%02X\n",
        idA,
        idB,
        idC);

    // =========================================================================
    // Verify ID
    // =========================================================================

    if (idA != ID_A ||
        idB != ID_B ||
        idC != ID_C)
    {
        printf(
            "GY271: unexpected device ID!\n");

        printf(
            "GY271: expected 0x48 0x34 0x33\n");

        return false;
    }

    printf(
        "GY271: HMC5883L identified successfully\n");

    // =========================================================================
    // Configure CONFIG_A
    // =========================================================================

    printf(
        "GY271: configuring CONFIG_A = 0x%02X\n",
        CONFIG_A_8_AVG_15HZ);

    if (!writeRegister(
            REG_CONFIG_A,
            CONFIG_A_8_AVG_15HZ))
    {
        printf(
            "GY271: CONFIG_A write failed\n");

        return false;
    }

    // =========================================================================
    // Configure CONFIG_B
    // =========================================================================

    printf(
        "GY271: configuring CONFIG_B = 0x%02X\n",
        CONFIG_B_GAIN_1_3GA);

    if (!writeRegister(
            REG_CONFIG_B,
            CONFIG_B_GAIN_1_3GA))
    {
        printf(
            "GY271: CONFIG_B write failed\n");

        return false;
    }

    // =========================================================================
    // Configure continuous measurement
    // =========================================================================

    printf(
        "GY271: configuring continuous measurement\n");

    if (!writeRegister(
            REG_MODE,
            MODE_CONTINUOUS))
    {
        printf(
            "GY271: MODE write failed\n");

        return false;
    }

    // =========================================================================
    // Allow sensor to start measurements
    // =========================================================================

    sleep_ms(100);

    // =========================================================================
    // Read configuration back
    // =========================================================================

    uint8_t configA = 0;
    uint8_t configB = 0;
    uint8_t mode = 0;

    if (!readRegister(
            REG_CONFIG_A,
            configA))
    {
        printf(
            "GY271: CONFIG_A read failed\n");

        return false;
    }

    if (!readRegister(
            REG_CONFIG_B,
            configB))
    {
        printf(
            "GY271: CONFIG_B read failed\n");

        return false;
    }

    if (!readRegister(
            REG_MODE,
            mode))
    {
        printf(
            "GY271: MODE read failed\n");

        return false;
    }

    printf(
        "GY271: CONFIG_A = 0x%02X\n",
        configA);

    printf(
        "GY271: CONFIG_B = 0x%02X\n",
        configB);

    printf(
        "GY271: MODE      = 0x%02X\n",
        mode);

    // =========================================================================
    // Initialization complete
    // =========================================================================

    initialized = true;

    printf(
        "GY271: initialization successful\n");

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

    // =========================================================================
    // Check data ready
    // =========================================================================

    uint8_t status = 0;

    if (!readRegister(
            REG_STATUS,
            status))
    {
        printf(
            "GY271: STATUS read failed\n");

        return false;
    }

    // =========================================================================
    // Check RDY bit
    // =========================================================================

    if (!(status & STATUS_RDY))
    {
        return false;
    }

    // =========================================================================
    // Read six data registers
    //
    // HMC5883L register order:
    //
    // 0x03 X MSB
    // 0x04 X LSB
    // 0x05 Z MSB
    // 0x06 Z LSB
    // 0x07 Y MSB
    // 0x08 Y LSB
    // =========================================================================

    uint8_t data[6] = {};

    if (!readRegisters(
            REG_DATA_X_MSB,
            data,
            sizeof(data)))
    {
        printf(
            "GY271: magnetic data read failed\n");

        return false;
    }

    // =========================================================================
    // X
    // =========================================================================

    x = static_cast<int16_t>(
        (static_cast<uint16_t>(data[0]) << 8) |
        data[1]);

    // =========================================================================
    // Z
    // =========================================================================

    z = static_cast<int16_t>(
        (static_cast<uint16_t>(data[2]) << 8) |
        data[3]);

    // =========================================================================
    // Y
    // =========================================================================

    y = static_cast<int16_t>(
        (static_cast<uint16_t>(data[4]) << 8) |
        data[5]);

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

    field.x =
        static_cast<float>(x) * scale;

    field.y =
        static_cast<float>(y) * scale;

    field.z =
        static_cast<float>(z) * scale;

    return true;
}

// =============================================================================
// Check Device Connection
// =============================================================================

bool GY271::isConnected()
{
    uint8_t idA = 0;
    uint8_t idB = 0;
    uint8_t idC = 0;

    if (!readRegister(
            REG_ID_A,
            idA))
    {
        return false;
    }

    if (!readRegister(
            REG_ID_B,
            idB))
    {
        return false;
    }

    if (!readRegister(
            REG_ID_C,
            idC))
    {
        return false;
    }

    return idA == ID_A &&
           idB == ID_B &&
           idC == ID_C;
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