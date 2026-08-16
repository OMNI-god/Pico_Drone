#pragma once

#include <cstdint>

#include "hardware/i2c.h"
#include "II2c.h"

class I2c : public II2c
{
public:
    I2c(
        i2c_inst_t *i2c,
        uint32_t baudrate,
        uint32_t sdaPin,
        uint32_t sclPin);

    bool initialize() override;

    int write(
        uint8_t address,
        const uint8_t *data,
        uint32_t length,
        bool nostop = false) override;

    int read(
        uint8_t address,
        uint8_t *data,
        uint32_t length,
        bool nostop = false) override;

    int writeRead(
        uint8_t address,
        const uint8_t *txData,
        uint32_t txLength,
        uint8_t *rxData,
        uint32_t rxLength) override;

private:
    i2c_inst_t *i2cInstance;

    uint32_t baudrate;
    uint32_t sdaPin;
    uint32_t sclPin;

    bool initialized = false;
};