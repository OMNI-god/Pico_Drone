#include "ADXL345.h"

#include "pico/stdlib.h"

#include <cstdio>

// ============================================================
// CONSTRUCTOR
// ============================================================

ADXL345::ADXL345(
    II2c &bus,
    uint8_t address)
    : bus(bus),
      address(address),
      initialized(false)
{
}

// ============================================================
// INITIALIZE
// ============================================================

bool ADXL345::initialize()
{
    initialized = false;

    printf("\n");
    printf("==============================\n");
    printf(" ADXL345 INITIALIZATION\n");
    printf("==============================\n");

    // --------------------------------------------------------
    // Detect device
    // --------------------------------------------------------

    if (!detectDevice())
    {
        printf(
            "ADXL345: device not found\n");

        return false;
    }

    printf(
        "ADXL345: found at 0x%02X\n",
        address);

    // --------------------------------------------------------
    // Read device ID
    // --------------------------------------------------------

    uint8_t deviceId = 0;

    if (!readDeviceId(deviceId))
    {
        printf(
            "ADXL345: failed to read DEVID\n");

        return false;
    }

    printf(
        "ADXL345: DEVID = 0x%02X\n",
        deviceId);

    if (deviceId != DEVICE_ID)
    {
        printf(
            "ADXL345: invalid device ID\n");

        return false;
    }

    // --------------------------------------------------------
    // DATA_FORMAT
    //
    // Bit 3 = FULL_RES
    //
    // Bit 1:0 = 00
    //
    // Full resolution
    // ±2g
    //
    // 0x08
    // --------------------------------------------------------

    printf(
        "ADXL345: configuring data format...\n");

    if (!writeRegister(
            REG_DATA_FORMAT,
            0x08))
    {
        printf(
            "ADXL345: DATA_FORMAT configuration failed\n");

        return false;
    }

    // --------------------------------------------------------
    // POWER_CTL
    //
    // Bit 3 = Measure
    //
    // 0x08 = measurement mode
    // --------------------------------------------------------

    printf(
        "ADXL345: enabling measurement mode...\n");

    if (!writeRegister(
            REG_POWER_CTL,
            0x08))
    {
        printf(
            "ADXL345: POWER_CTL configuration failed\n");

        return false;
    }

    sleep_ms(10);

    initialized = true;

    printf(
        "ADXL345: initialization successful\n");

    return true;
}

// ============================================================
// DETECT DEVICE
// ============================================================

bool ADXL345::detectDevice()
{
    uint8_t originalAddress = address;

    uint8_t deviceId = 0;

    // --------------------------------------------------------
    // Try configured address
    // --------------------------------------------------------

    printf(
        "ADXL345: checking address 0x%02X...\n",
        address);

    if (readDeviceId(deviceId))
    {
        printf(
            "ADXL345: address 0x%02X returned 0x%02X\n",
            address,
            deviceId);

        if (deviceId == DEVICE_ID)
        {
            return true;
        }
    }
    else
    {
        printf(
            "ADXL345: no response at 0x%02X\n",
            address);
    }

    // --------------------------------------------------------
    // Try alternate address
    // --------------------------------------------------------

    if (originalAddress == ADDRESS_LOW)
    {
        address = ADDRESS_HIGH;
    }
    else
    {
        address = ADDRESS_LOW;
    }

    printf(
        "ADXL345: checking address 0x%02X...\n",
        address);

    deviceId = 0;

    if (readDeviceId(deviceId))
    {
        printf(
            "ADXL345: address 0x%02X returned 0x%02X\n",
            address,
            deviceId);

        if (deviceId == DEVICE_ID)
        {
            return true;
        }
    }
    else
    {
        printf(
            "ADXL345: no response at 0x%02X\n",
            address);
    }

    // --------------------------------------------------------
    // Nothing found
    // --------------------------------------------------------

    address = originalAddress;

    return false;
}

// ============================================================
// READ DEVICE ID
// ============================================================

bool ADXL345::readDeviceId(
    uint8_t &deviceId)
{
    return readRegister(
        REG_DEVID,
        deviceId);
}

// ============================================================
// READ ACCELERATION
// ============================================================

bool ADXL345::readAcceleration(
    Acceleration &acceleration)
{
    if (!initialized)
    {
        return false;
    }

    uint8_t buffer[6];

    if (!readRegisters(
            REG_DATAX0,
            buffer,
            sizeof(buffer)))
    {
        return false;
    }

    // --------------------------------------------------------
    // ADXL345 data format:
    //
    // X0 = low byte
    // X1 = high byte
    //
    // Y0 = low byte
    // Y1 = high byte
    //
    // Z0 = low byte
    // Z1 = high byte
    // --------------------------------------------------------

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

    // --------------------------------------------------------
    // Convert to g
    // --------------------------------------------------------

    acceleration.x =
        static_cast<float>(x) *
        ACCEL_SCALE;

    acceleration.y =
        static_cast<float>(y) *
        ACCEL_SCALE;

    acceleration.z =
        static_cast<float>(z) *
        ACCEL_SCALE;

    return true;
}

// ============================================================
// CHECK CONNECTION
// ============================================================

bool ADXL345::isConnected()
{
    uint8_t deviceId = 0;

    if (!readDeviceId(deviceId))
    {
        return false;
    }

    return deviceId == DEVICE_ID;
}

// ============================================================
// GET ADDRESS
// ============================================================

uint8_t ADXL345::getAddress() const
{
    return address;
}

// ============================================================
// READ REGISTER
// ============================================================

bool ADXL345::readRegister(
    uint8_t reg,
    uint8_t &value)
{
    return readRegisters(
        reg,
        &value,
        1);
}

// ============================================================
// WRITE REGISTER
// ============================================================

bool ADXL345::writeRegister(
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
           static_cast<int>(
               sizeof(buffer));
}

// ============================================================
// READ MULTIPLE REGISTERS
// ============================================================

bool ADXL345::readRegisters(
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

    // --------------------------------------------------------
    // ADXL345 multi-byte read requires bit 6 of the register
    // address to be set.
    //
    // 0x32 -> 0xF2
    //
    // This enables multi-byte access.
    // --------------------------------------------------------

    uint8_t multiByteReg =
        reg | 0x40;

    const int result =
        bus.writeRead(
            address,
            &multiByteReg,
            1,
            data,
            length);

    return result ==
           static_cast<int>(length);
}

// ============================================================
// MAKE SIGNED 16-BIT VALUE
// ============================================================

int16_t ADXL345::makeInt16(
    uint8_t low,
    uint8_t high)
{
    const uint16_t value =
        static_cast<uint16_t>(
            static_cast<uint16_t>(low) |
            (static_cast<uint16_t>(high) << 8));

    return static_cast<int16_t>(value);
}