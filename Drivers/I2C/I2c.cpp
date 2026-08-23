#include "I2c.h"

#include "hardware/gpio.h"

#include <cstdio>

// =============================================================================
// Constructor
// =============================================================================

I2c::I2c(
    i2c_inst_t *i2c,
    uint32_t baudrate,
    uint32_t sdaPin,
    uint32_t sclPin)
    : i2cInstance(i2c),
      baudrate(baudrate),
      sdaPin(sdaPin),
      sclPin(sclPin)
{
}

// =============================================================================
// Initialize
// =============================================================================

bool I2c::initialize()
{
    if (initialized)
    {
        return true;
    }

    if (i2cInstance == nullptr)
    {
        printf("I2C: invalid instance\n");
        return false;
    }

    i2c_init(
        i2cInstance,
        baudrate);

    gpio_set_function(
        sdaPin,
        GPIO_FUNC_I2C);

    gpio_set_function(
        sclPin,
        GPIO_FUNC_I2C);

    gpio_pull_up(sdaPin);
    gpio_pull_up(sclPin);

    initialized = true;

    printf(
        "I2C initialized: SDA=%lu SCL=%lu Baudrate=%lu\n",
        static_cast<unsigned long>(sdaPin),
        static_cast<unsigned long>(sclPin),
        static_cast<unsigned long>(baudrate));

    return true;
}

// =============================================================================
// Write
// =============================================================================

int I2c::write(
    uint8_t address,
    const uint8_t *data,
    uint32_t length,
    bool nostop)
{
    if (!initialized)
    {
        printf("I2C: write called before initialize\n");
        return -1;
    }

    if (data == nullptr || length == 0)
    {
        printf("I2C: invalid write parameters\n");
        return -1;
    }

    int result =
        i2c_write_blocking(
            i2cInstance,
            address,
            data,
            length,
            nostop);

    if (result < 0)
    {
        printf(
            "I2C: write failed, address=0x%02X result=%d\n",
            address,
            result);
    }

    return result;
}

// =============================================================================
// Read
// =============================================================================

int I2c::read(
    uint8_t address,
    uint8_t *data,
    uint32_t length,
    bool nostop)
{
    if (!initialized)
    {
        printf("I2C: read called before initialize\n");
        return -1;
    }

    if (data == nullptr || length == 0)
    {
        printf("I2C: invalid read parameters\n");
        return -1;
    }

    int result =
        i2c_read_blocking(
            i2cInstance,
            address,
            data,
            length,
            nostop);

    if (result < 0)
    {
        printf(
            "I2C: read failed, address=0x%02X result=%d\n",
            address,
            result);
    }

    return result;
}

// =============================================================================
// Write + Read
//
// Performs:
//
// START
// ADDRESS + WRITE
// REGISTER
// REPEATED START
// ADDRESS + READ
// DATA
// STOP
//
// This is the transaction required by HMC5883L.
// =============================================================================

int I2c::writeRead(
    uint8_t address,
    const uint8_t *txData,
    uint32_t txLength,
    uint8_t *rxData,
    uint32_t rxLength)
{
    if (!initialized)
    {
        printf(
            "I2C: writeRead called before initialize\n");

        return -1;
    }

    if (txData == nullptr ||
        rxData == nullptr ||
        txLength == 0 ||
        rxLength == 0)
    {
        printf(
            "I2C: invalid writeRead parameters\n");

        return -1;
    }

    // -------------------------------------------------------------------------
    // Write register address.
    //
    // nostop = true
    //
    // This keeps the bus active and generates a repeated START before read.
    // -------------------------------------------------------------------------

    int writeResult =
        i2c_write_blocking(
            i2cInstance,
            address,
            txData,
            txLength,
            true);

    if (writeResult != static_cast<int>(txLength))
    {
        printf(
            "I2C: writeRead WRITE failed\n");

        printf(
            "      address = 0x%02X\n",
            address);

        printf(
            "      expected = %lu\n",
            static_cast<unsigned long>(txLength));

        printf(
            "      actual = %d\n",
            writeResult);

        return -1;
    }

    // -------------------------------------------------------------------------
    // Read data.
    //
    // nostop = false
    //
    // This generates STOP after the read.
    // -------------------------------------------------------------------------

    int readResult =
        i2c_read_blocking(
            i2cInstance,
            address,
            rxData,
            rxLength,
            false);

    if (readResult != static_cast<int>(rxLength))
    {
        printf(
            "I2C: writeRead READ failed\n");

        printf(
            "      address = 0x%02X\n",
            address);

        printf(
            "      expected = %lu\n",
            static_cast<unsigned long>(rxLength));

        printf(
            "      actual = %d\n",
            readResult);

        return -1;
    }

    return readResult;
}