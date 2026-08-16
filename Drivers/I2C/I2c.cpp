#include "I2c.h"

#include "hardware/gpio.h"

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

bool I2c::initialize()
{
    if (initialized)
    {
        return true;
    }

    if (i2cInstance == nullptr)
    {
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

    return true;
}

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

    if (data == nullptr || length == 0)
    {
        return -1;
    }

    return i2c_write_blocking(
        i2cInstance,
        address,
        data,
        length,
        nostop);
}

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

    if (data == nullptr || length == 0)
    {
        return -1;
    }

    return i2c_read_blocking(
        i2cInstance,
        address,
        data,
        length,
        nostop);
}

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

    return i2c_write_blocking(
               i2cInstance,
               address,
               txData,
               txLength,
               true) == static_cast<int>(txLength)
               ? i2c_read_blocking(
                     i2cInstance,
                     address,
                     rxData,
                     rxLength,
                     false)
               : -1;
}