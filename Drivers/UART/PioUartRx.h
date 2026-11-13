#pragma once

#include <cstdint>

#include "hardware/pio.h"

/*
 * PIO-based UART receiver.
 *
 * The RP2350 only has two hardware UARTs. The GPS
 * module is fixed to GPIO8/9 (UART1) and the ELRS
 * receiver to GPIO4/5 (also UART1), so the GPS
 * receive path is implemented in PIO instead.
 *
 * The state machine samples 8N1 at the configured
 * baud rate and pushes each received byte into the
 * RX FIFO. A PIO interrupt drains that FIFO into a
 * software ring buffer so the telemetry task can
 * consume the NMEA stream at its own rate without
 * overrunning the 8-entry PIO FIFO.
 *
 * Only RX is implemented: the GPS receiver never
 * needs to be written to in this design.
 */
class PioUartRx
{
public:
    explicit PioUartRx(
        std::uint32_t rxPin,
        PIO pio = pio0);

    /*
     * Load the PIO program, configure the pin and
     * enable the RX interrupt.
     */
    bool initialize(
        std::uint32_t baudrate);

    /*
     * True when at least one received byte is
     * waiting in the ring buffer.
     */
    bool available() const;

    /*
     * Pop one received byte.
     *
     * @return Byte value (0..255), or -1 when the
     *         ring buffer is empty.
     */
    int read();

    std::uint32_t getRxPin() const;

    std::uint32_t getReceivedBytes() const;

private:
    static constexpr std::uint32_t BufferSize = 1024;
    static constexpr std::uint32_t BufferMask = BufferSize - 1;

    static void interruptHandler();

    void drainFifo();

private:
    PIO pio_;
    std::int32_t sm_ = -1;

    std::uint32_t rxPin_;
    std::uint32_t offset_ = 0;

    bool initialized_ = false;

    volatile std::uint8_t buffer_[BufferSize] = {};

    volatile std::uint32_t head_ = 0;
    volatile std::uint32_t tail_ = 0;

    volatile std::uint32_t receivedBytes_ = 0;

    static PioUartRx *instance_;
};
