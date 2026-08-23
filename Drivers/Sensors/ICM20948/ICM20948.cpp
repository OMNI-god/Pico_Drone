#include "ICM20948.h"

#include "pico/stdlib.h"

#include <cstdio>

// =============================================================================
// CONSTRUCTOR
// =============================================================================

ICM20948::ICM20948(
    II2c &bus,
    uint8_t address)
    : bus(bus),
      address(address),
      initialized(false),
      currentBank(0xFF),
      accelScale(ACCEL_SCALE),
      gyroScale(GYRO_SCALE)
{
}

// =============================================================================
// INITIALIZE
// =============================================================================

bool ICM20948::initialize()
{
    initialized = false;

    currentBank = 0xFF;

    printf("\n");
    printf("==============================\n");
    printf(" ICM20948 INITIALIZATION\n");
    printf("==============================\n");

    // -------------------------------------------------------------------------
    // Detect sensor
    // -------------------------------------------------------------------------

    printf(
        "ICM20948: checking address 0x%02X...\n",
        address);

    if (!detectDevice())
    {
        printf(
            "ICM20948: device not found\n");

        return false;
    }

    printf(
        "ICM20948: found at 0x%02X\n",
        address);

    // -------------------------------------------------------------------------
    // Select Bank 0
    // -------------------------------------------------------------------------

    if (!selectBank(0))
    {
        printf(
            "ICM20948: failed to select bank 0\n");

        return false;
    }

    // -------------------------------------------------------------------------
    // Read WHO_AM_I
    // -------------------------------------------------------------------------

    uint8_t whoAmI = 0;

    if (!readWhoAmI(whoAmI))
    {
        printf(
            "ICM20948: WHO_AM_I read failed\n");

        return false;
    }

    printf(
        "ICM20948: WHO_AM_I = 0x%02X\n",
        whoAmI);

    if (whoAmI != DEVICE_ID)
    {
        printf(
            "ICM20948: wrong device ID\n");

        return false;
    }

    // =========================================================================
    // SOFTWARE RESET
    // =========================================================================

    printf(
        "ICM20948: resetting...\n");

    if (!writeRegister(
            REG_PWR_MGMT_1,
            0x80))
    {
        printf(
            "ICM20948: reset failed\n");

        return false;
    }

    sleep_ms(100);

    // The reset changes the register state.
    currentBank = 0xFF;

    // -------------------------------------------------------------------------
    // Select bank 0 again
    // -------------------------------------------------------------------------

    if (!selectBank(0))
    {
        printf(
            "ICM20948: bank 0 selection failed after reset\n");

        return false;
    }

    // -------------------------------------------------------------------------
    // Verify WHO_AM_I after reset
    // -------------------------------------------------------------------------

    if (!readWhoAmI(whoAmI))
    {
        printf(
            "ICM20948: WHO_AM_I failed after reset\n");

        return false;
    }

    if (whoAmI != DEVICE_ID)
    {
        printf(
            "ICM20948: invalid WHO_AM_I after reset: 0x%02X\n",
            whoAmI);

        return false;
    }

    // =========================================================================
    // WAKE SENSOR
    // =========================================================================

    printf(
        "ICM20948: waking sensor...\n");

    /*
     * PWR_MGMT_1
     *
     * SLEEP = 0
     * CLKSEL = 001
     *
     * 0x01
     */

    if (!writeRegister(
            REG_PWR_MGMT_1,
            0x01))
    {
        printf(
            "ICM20948: wake failed\n");

        return false;
    }

    sleep_ms(10);

    // =========================================================================
    // ENABLE ACCELEROMETER + GYROSCOPE
    // =========================================================================

    /*
     * PWR_MGMT_2
     *
     * 0 = enabled
     */

    if (!writeRegister(
            REG_PWR_MGMT_2,
            0x00))
    {
        printf(
            "ICM20948: sensor enable failed\n");

        return false;
    }

    sleep_ms(10);

    // =========================================================================
    // BANK 2
    // =========================================================================

    if (!selectBank(2))
    {
        printf(
            "ICM20948: bank 2 selection failed\n");

        return false;
    }

    // =========================================================================
    // GYROSCOPE CONFIGURATION
    // =========================================================================

    /*
     * GYRO_CONFIG_1
     *
     * FS_SEL = 0
     * ±250 DPS
     */

    if (!writeRegister(
            REG_GYRO_CONFIG_1,
            0x00))
    {
        printf(
            "ICM20948: gyro configuration failed\n");

        return false;
    }

    // =========================================================================
    // ACCELEROMETER CONFIGURATION
    // =========================================================================

    /*
     * ACCEL_CONFIG
     *
     * FS_SEL = 0
     * ±2g
     */

    if (!writeRegister(
            REG_ACCEL_CONFIG,
            0x00))
    {
        printf(
            "ICM20948: accelerometer configuration failed\n");

        return false;
    }

    sleep_ms(10);

    // =========================================================================
    // RETURN TO BANK 0
    // =========================================================================

    if (!selectBank(0))
    {
        printf(
            "ICM20948: failed to return to bank 0\n");

        return false;
    }

    // =========================================================================
    // INITIALIZATION COMPLETE
    // =========================================================================

    initialized = true;

    printf(
        "ICM20948: initialization successful\n");

    return true;
}

// =============================================================================
// DETECT DEVICE
// =============================================================================

bool ICM20948::detectDevice()
{
    uint8_t originalAddress = address;

    uint8_t whoAmI = 0;

    // -------------------------------------------------------------------------
    // Try configured address
    // -------------------------------------------------------------------------

    currentBank = 0xFF;

    if (readWhoAmI(whoAmI))
    {
        printf(
            "ICM20948: address 0x%02X returned 0x%02X\n",
            address,
            whoAmI);

        if (whoAmI == DEVICE_ID)
        {
            return true;
        }
    }

    // -------------------------------------------------------------------------
    // Try alternate address
    // -------------------------------------------------------------------------

    if (originalAddress == ADDRESS_69)
    {
        address = ADDRESS_68;
    }
    else
    {
        address = ADDRESS_69;
    }

    currentBank = 0xFF;

    printf(
        "ICM20948: trying alternate address 0x%02X...\n",
        address);

    if (readWhoAmI(whoAmI))
    {
        printf(
            "ICM20948: address 0x%02X returned 0x%02X\n",
            address,
            whoAmI);

        if (whoAmI == DEVICE_ID)
        {
            return true;
        }
    }

    // -------------------------------------------------------------------------
    // Restore original address
    // -------------------------------------------------------------------------

    address = originalAddress;
    currentBank = 0xFF;

    return false;
}

// =============================================================================
// READ WHO AM I
// =============================================================================

bool ICM20948::readWhoAmI(
    uint8_t &value)
{
    // WHO_AM_I is in bank 0.

    if (!selectBank(0))
    {
        return false;
    }

    return readRegister(
        REG_WHO_AM_I,
        value);
}

// =============================================================================
// READ ACCELEROMETER
// =============================================================================

bool ICM20948::readAcceleration(
    Acceleration &acceleration)
{
    if (!initialized)
    {
        return false;
    }

    if (!selectBank(0))
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
        makeInt16(
            buffer[0],
            buffer[1]);

    const int16_t y =
        makeInt16(
            buffer[2],
            buffer[3]);

    const int16_t z =
        makeInt16(
            buffer[4],
            buffer[5]);

    acceleration.x =
        static_cast<float>(x) *
        accelScale;

    acceleration.y =
        static_cast<float>(y) *
        accelScale;

    acceleration.z =
        static_cast<float>(z) *
        accelScale;

    return true;
}

// =============================================================================
// READ GYROSCOPE
// =============================================================================

bool ICM20948::readGyroscope(
    Gyroscope &gyroscope)
{
    if (!initialized)
    {
        return false;
    }

    if (!selectBank(0))
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
        makeInt16(
            buffer[0],
            buffer[1]);

    const int16_t y =
        makeInt16(
            buffer[2],
            buffer[3]);

    const int16_t z =
        makeInt16(
            buffer[4],
            buffer[5]);

    gyroscope.x =
        static_cast<float>(x) *
        gyroScale;

    gyroscope.y =
        static_cast<float>(y) *
        gyroScale;

    gyroscope.z =
        static_cast<float>(z) *
        gyroScale;

    return true;
}

// =============================================================================
// READ ACCELEROMETER + GYROSCOPE
// =============================================================================

bool ICM20948::read(
    SensorData &data)
{
    if (!initialized)
    {
        return false;
    }

    if (!selectBank(0))
    {
        return false;
    }

    /*
     * Starting at 0x2D:
     *
     * 0x2D - 0x32 = Accelerometer
     * 0x33 - 0x38 = Gyroscope
     *
     * Total = 12 bytes
     */

    uint8_t buffer[12];

    if (!readRegisters(
            REG_ACCEL_XOUT_H,
            buffer,
            sizeof(buffer)))
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Accelerometer
    // -------------------------------------------------------------------------

    const int16_t ax =
        makeInt16(
            buffer[0],
            buffer[1]);

    const int16_t ay =
        makeInt16(
            buffer[2],
            buffer[3]);

    const int16_t az =
        makeInt16(
            buffer[4],
            buffer[5]);

    // -------------------------------------------------------------------------
    // Gyroscope
    // -------------------------------------------------------------------------

    const int16_t gx =
        makeInt16(
            buffer[6],
            buffer[7]);

    const int16_t gy =
        makeInt16(
            buffer[8],
            buffer[9]);

    const int16_t gz =
        makeInt16(
            buffer[10],
            buffer[11]);

    // -------------------------------------------------------------------------
    // Convert accelerometer
    // -------------------------------------------------------------------------

    data.acceleration.x =
        static_cast<float>(ax) *
        accelScale;

    data.acceleration.y =
        static_cast<float>(ay) *
        accelScale;

    data.acceleration.z =
        static_cast<float>(az) *
        accelScale;

    // -------------------------------------------------------------------------
    // Convert gyroscope
    // -------------------------------------------------------------------------

    data.gyroscope.x =
        static_cast<float>(gx) *
        gyroScale;

    data.gyroscope.y =
        static_cast<float>(gy) *
        gyroScale;

    data.gyroscope.z =
        static_cast<float>(gz) *
        gyroScale;

    return true;
}

// =============================================================================
// IS CONNECTED
// =============================================================================

bool ICM20948::isConnected()
{
    uint8_t whoAmI = 0;

    if (!readWhoAmI(whoAmI))
    {
        return false;
    }

    return whoAmI == DEVICE_ID;
}

// =============================================================================
// GET ADDRESS
// =============================================================================

uint8_t ICM20948::getAddress() const
{
    return address;
}

// =============================================================================
// SELECT REGISTER BANK
// =============================================================================

bool ICM20948::selectBank(
    uint8_t bank)
{
    bank &= 0x03;

    if (currentBank == bank)
    {
        return true;
    }

    const uint8_t value =
        static_cast<uint8_t>(
            bank << 4);

    const uint8_t buffer[2] =
        {
            REG_BANK_SEL,
            value};

    const int result =
        bus.write(
            address,
            buffer,
            sizeof(buffer));

    if (result !=
        static_cast<int>(sizeof(buffer)))
    {
        return false;
    }

    currentBank = bank;

    return true;
}

// =============================================================================
// READ REGISTER
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
// WRITE REGISTER
// =============================================================================

bool ICM20948::writeRegister(
    uint8_t reg,
    uint8_t value)
{
    const uint8_t buffer[2] =
        {
            reg,
            value};

    const int result =
        bus.write(
            address,
            buffer,
            sizeof(buffer));

    return result ==
           static_cast<int>(sizeof(buffer));
}

// =============================================================================
// READ MULTIPLE REGISTERS
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

// =============================================================================
// CONVERT TWO BYTES TO SIGNED 16-BIT
// =============================================================================

int16_t ICM20948::makeInt16(
    uint8_t high,
    uint8_t low)
{
    uint16_t value =
        static_cast<uint16_t>(
            (static_cast<uint16_t>(high) << 8) |
            low);

    return static_cast<int16_t>(value);
}