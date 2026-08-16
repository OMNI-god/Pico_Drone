#pragma once

#include <cstdint>

class II2c
{
public:
    virtual ~II2c() = default;

    virtual bool initialize() = 0;

    virtual int write(
        uint8_t address,
        const uint8_t *data,
        uint32_t length,
        bool nostop = false) = 0;

    virtual int read(
        uint8_t address,
        uint8_t *data,
        uint32_t length,
        bool nostop = false) = 0;

    virtual int writeRead(
        uint8_t address,
        const uint8_t *txData,
        uint32_t txLength,
        uint8_t *rxData,
        uint32_t rxLength) = 0;
};