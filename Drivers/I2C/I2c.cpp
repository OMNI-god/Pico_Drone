#include "I2c.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

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

    sleep_us(100);

    initialized = true;

    printf(
        "I2C initialized: SDA=%lu SCL=%lu Baudrate=%lu\n",
        static_cast<unsigned long>(sdaPin),
        static_cast<unsigned long>(sclPin),
        static_cast<unsigned long>(baudrate));

    return true;
}

// =============================================================================
// Recover I2C bus
// =============================================================================

bool I2c::recoverBus()
{
    if (i2cInstance == nullptr)
    {
        return false;
    }

    printf("I2C: recovering bus...\n");

    // Disable I2C peripheral.
    i2c_deinit(i2cInstance);

    // Temporarily control pins manually.
    gpio_set_function(
        sdaPin,
        GPIO_FUNC_SIO);

    gpio_set_function(
        sclPin,
        GPIO_FUNC_SIO);

    gpio_set_dir(
        sdaPin,
        GPIO_IN);

    gpio_set_dir(
        sclPin,
        GPIO_OUT);

    gpio_put(
        sclPin,
        1);

    gpio_pull_up(sdaPin);
    gpio_pull_up(sclPin);

    sleep_us(10);

    // Generate up to 9 clock pulses.
    //
    // This allows a slave that is stuck waiting for clock pulses
    // to release SDA.
    for (int i = 0; i < 9; ++i)
    {
        gpio_put(
            sclPin,
            0);

        sleep_us(5);

        gpio_put(
            sclPin,
            1);

        sleep_us(5);

        if (gpio_get(sdaPin))
        {
            break;
        }
    }

    // Generate STOP:
    //
    // SDA LOW
    // SCL HIGH
    // SDA HIGH
    gpio_set_dir(
        sdaPin,
        GPIO_OUT);

    gpio_put(
        sdaPin,
        0);

    sleep_us(5);

    gpio_put(
        sclPin,
        1);

    sleep_us(5);

    gpio_put(
        sdaPin,
        1);

    sleep_us(5);

    // Restore I2C peripheral.
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

    sleep_us(100);

    printf("I2C: bus recovery complete\n");

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
        return -1;
    }

    if (data == nullptr ||
        length == 0)
    {
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
            "I2C WRITE ERROR: addr=0x%02X result=%d\n",
            address,
            result);

        recoverBus();

        return result;
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
        return -1;
    }

    if (data == nullptr ||
        length == 0)
    {
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
            "I2C READ ERROR: addr=0x%02X result=%d\n",
            address,
            result);

        recoverBus();

        return result;
    }

    return result;
}

// =============================================================================
// Write + Read
//
// IMPORTANT:
//
// BMP280 register read:
//
// START
//   ADDRESS + WRITE
//   REGISTER
// REPEATED START
//   ADDRESS + READ
//   DATA...
// STOP
//
// `nostop=true` on the write is essential.
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
        return -1;
    }

    if (txData == nullptr ||
        rxData == nullptr ||
        txLength == 0 ||
        rxLength == 0)
    {
        return -1;
    }

    // -------------------------------------------------------------------------
    // Write register address without STOP.
    // -------------------------------------------------------------------------

    int writeResult =
        i2c_write_blocking(
            i2cInstance,
            address,
            txData,
            txLength,
            true);

    if (writeResult !=
        static_cast<int>(txLength))
    {
        printf(
            "I2C WRITE-READ: write failed "
            "addr=0x%02X result=%d expected=%lu\n",
            address,
            writeResult,
            static_cast<unsigned long>(txLength));

        recoverBus();

        return -1;
    }

    // -------------------------------------------------------------------------
    // Small delay before read.
    // -------------------------------------------------------------------------

    sleep_us(10);

    // -------------------------------------------------------------------------
    // Read data.
    //
    // nostop=false generates STOP.
    // -------------------------------------------------------------------------

    int readResult =
        i2c_read_blocking(
            i2cInstance,
            address,
            rxData,
            rxLength,
            false);

    if (readResult !=
        static_cast<int>(rxLength))
    {
        printf(
            "I2C WRITE-READ: read failed "
            "addr=0x%02X result=%d expected=%lu\n",
            address,
            readResult,
            static_cast<unsigned long>(rxLength));

        recoverBus();

        return -1;
    }

    return readResult;
}