#pragma once

#include "IUart.h"

class Elrs : public IUart
{
public:
    Elrs(uart_inst_t *uart, uint32_t baudrate, uint32_t rxPin, uint32_t txPin);
    bool initialize() override;
    int write(const uint8_t *data, uint32_t length) override;
    int read(uint8_t *data, uint32_t length) override;
};