#include "PioUartRx.h"

#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

#include "uart_rx.pio.h"

PioUartRx *PioUartRx::instance_ = nullptr;

// =============================================================================
// Constructor
// =============================================================================

PioUartRx::PioUartRx(
    std::uint32_t rxPin,
    PIO pio)
    : pio_(pio),
      rxPin_(rxPin)
{
}

// =============================================================================
// Initialize
// =============================================================================

bool PioUartRx::initialize(
    std::uint32_t baudrate)
{
    if (initialized_)
    {
        return true;
    }

    if (baudrate == 0)
    {
        return false;
    }

    /*
     * The interrupt handler has to reach this
     * instance. There is exactly one GPS receiver,
     * so a single static pointer is sufficient.
     */

    instance_ = this;

    sm_ = pio_claim_unused_sm(
        pio_,
        false);

    if (sm_ < 0)
    {
        return false;
    }

    offset_ = pio_add_program(
        pio_,
        &uart_rx_program);

    uart_rx_program_init(
        pio_,
        static_cast<uint>(sm_),
        offset_,
        rxPin_,
        baudrate);

    /*
     * Interrupt when the PIO RX FIFO has data.
     *
     * The FIFO is only 8 entries deep, so it must
     * be drained by the interrupt handler rather
     * than by the (much slower) telemetry task.
     */

    pio_set_irq0_source_enabled(
        pio_,
        pio_get_rx_fifo_not_empty_interrupt_source(
            static_cast<uint>(sm_)),
        true);

    const uint irqNumber =
        PIO_IRQ_NUM(pio_, 0);

    irq_set_exclusive_handler(
        irqNumber,
        interruptHandler);

    irq_set_enabled(
        irqNumber,
        true);

    initialized_ = true;

    return true;
}

// =============================================================================
// Status
// =============================================================================

bool PioUartRx::available() const
{
    return head_ != tail_;
}

std::uint32_t PioUartRx::getRxPin() const
{
    return rxPin_;
}

std::uint32_t PioUartRx::getReceivedBytes() const
{
    return receivedBytes_;
}

// =============================================================================
// Read one byte
// =============================================================================

int PioUartRx::read()
{
    if (tail_ == head_)
    {
        return -1;
    }

    /*
     * The producer writes the byte and then
     * publishes it by advancing head_. Make sure
     * the data write is visible before reading it.
     */

    __dmb();

    const std::uint8_t value =
        buffer_[tail_ & BufferMask];

    tail_ = tail_ + 1;

    return static_cast<int>(value);
}

// =============================================================================
// Interrupt handler
// =============================================================================

void PioUartRx::interruptHandler()
{
    if (instance_ != nullptr)
    {
        instance_->drainFifo();
    }
}

// =============================================================================
// Drain the PIO RX FIFO into the ring buffer
// =============================================================================

void PioUartRx::drainFifo()
{
    while (!pio_sm_is_rx_fifo_empty(
        pio_,
        static_cast<uint>(sm_)))
    {
        const std::uint32_t word =
            pio_sm_get(
                pio_,
                static_cast<uint>(sm_));

        /*
         * The PIO program shifts each bit into the
         * ISR from the MSB, so the received byte is
         * left-justified in the FIFO word.
         */

        const std::uint8_t value =
            static_cast<std::uint8_t>(
                (word >> 24u) & 0xFFu);

        ++receivedBytes_;

        if ((head_ - tail_) < BufferSize)
        {
            buffer_[head_ & BufferMask] = value;

            /*
             * Publish the byte only after it has been
             * written to the ring buffer.
             */

            __dmb();

            head_ = head_ + 1;
        }
    }
}
