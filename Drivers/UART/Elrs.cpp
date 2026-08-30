#include "Elrs.h"

Elrs::Elrs(
    uart_inst_t *uart,
    uint32_t baudrate,
    uint32_t rxPin,
    uint32_t txPin)
{
    uartInstance = uart;
    this->baudrate = baudrate;
    this->rxPin = rxPin;
    this->txPin = txPin;
}

bool Elrs::initialize()
{
    if (initialized)
        return true;

    uart_init(
        uartInstance,
        baudrate);

    gpio_set_function(
        txPin,
        GPIO_FUNC_UART);

    gpio_set_function(
        rxPin,
        GPIO_FUNC_UART);

    initialized = true;

    return true;
}

int Elrs::write(
    const uint8_t *data,
    uint32_t length)
{
    if (!initialized)
        return -1;

    uart_write_blocking(
        uartInstance,
        data,
        length);

    return length;
}

int Elrs::read(
    uint8_t *data,
    uint32_t length)
{
    if (!initialized)
        return -1;

    uint32_t count = 0;

    while (
        count < length &&
        uart_is_readable(uartInstance))
    {
        data[count++] = uart_getc(uartInstance);
    }

    return count;
}

bool Elrs::available()
{
    if (!initialized)
        return false;

    return uart_is_readable(uartInstance);
}