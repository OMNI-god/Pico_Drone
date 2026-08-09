#pragma once

#include "hardware/i2c.h"

class II2c
{
protected:
    i2c_inst_t *i2cInstance;
    uint32_t baudrate;
    uint32_t sdaPin;
    uint32_t sclPin;
    bool initialized = false;

public:
    virtual ~II2c() = default;

    virtual bool initialize() = 0;

    virtual int write(
        uint8_t address,
        const uint8_t *data,
        uint32_t length) = 0;

    virtual int read(
        uint8_t address,
        uint8_t *data,
        uint32_t length) = 0;
};