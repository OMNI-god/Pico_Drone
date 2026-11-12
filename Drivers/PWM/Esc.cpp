#include "Esc.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

Esc::Esc(
    uint32_t pinNumber,
    uint32_t frequency)
{
    pin = pinNumber;
    this->frequency = frequency;

    /*
     * The PWM slice is initialized lazily by the first
     * setPulseWidth() call, or explicitly through
     * initialize().
     *
     * This keeps global objects away from hardware
     * access before main() runs.
     */
}

bool Esc::initialize()
{
    gpio_set_function(
        pin,
        GPIO_FUNC_PWM);

    sliceNumber =
        pwm_gpio_to_slice_num(pin);

    channel =
        pwm_gpio_to_channel(pin);

    pwm_config config =
        pwm_get_default_config();

    clkDiv = 64.0f;

    pwm_config_set_clkdiv(
        &config,
        clkDiv);

    /*
     * PWM period:
     *
     *     (wrap + 1) / (clk_sys / clkDiv) = 1 / frequency
     *
     * Therefore:
     *
     *     wrap = clk_sys / (clkDiv * frequency) - 1
     */

    wrap =
        static_cast<uint32_t>(
            static_cast<float>(clock_get_hz(clk_sys)) /
            (clkDiv * static_cast<float>(frequency))) -
        1u;

    pwm_config_set_wrap(
        &config,
        static_cast<uint16_t>(wrap));

    pwm_init(
        sliceNumber,
        &config,
        true);

    initialized = true;

    return true;
}

int Esc::setSpeed(
    int pulseWidthUs)
{
    if (!isValidTiming(
            static_cast<uint32_t>(pulseWidthUs)))
    {
        return -1;
    }

    return static_cast<int>(
        setPulseWidth(
            static_cast<uint32_t>(pulseWidthUs)));
}

bool Esc::isValidTiming(
    uint32_t pulseWidthUs)
{
    return pulseWidthUs >= 900 &&
           pulseWidthUs <= 2100;
}

uint32_t Esc::setPulseWidth(
    uint32_t width)
{
    if (!initialized)
    {
        if (!initialize())
            return 0;
    }

    /*
     * Keep the pulse inside the range accepted by
     * virtually every ESC / servo.
     */

    if (width < 800u)
    {
        width = 800u;
    }

    if (width > 2200u)
    {
        width = 2200u;
    }

    const float pwmClock =
        static_cast<float>(clock_get_hz(clk_sys)) /
        clkDiv;

    const uint32_t level =
        static_cast<uint32_t>(
            (static_cast<float>(width) * pwmClock) /
            1000000.0f);

    pwm_set_chan_level(
        sliceNumber,
        channel,
        static_cast<uint16_t>(level));

    return level;
}