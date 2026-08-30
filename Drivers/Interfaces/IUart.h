#pragma once

#include "hardware/uart.h"
#include "hardware/gpio.h"

class IUart
{
protected:
    uart_inst_t *uartInstance;
    uint32_t baudrate;
    uint32_t rxPin;
    uint32_t txPin;
    bool initialized = false;

public:
    virtual ~IUart() = default;

    virtual bool initialize() = 0;

    virtual int write(
        const uint8_t *data,
        uint32_t length) = 0;

    virtual int read(
        uint8_t *data,
        uint32_t length) = 0;

    virtual bool available() = 0;
};