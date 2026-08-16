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
    initialized = initialize();
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

    pwm_set_enabled(
        sliceNumber,
        true);

    wrap =
        clock_get_hz(clk_sys);

    wrap /=
        frequency - 1;

    pwm_set_wrap(
        sliceNumber,
        wrap);

    return true;
}

int Esc::setSpeed(
    int throttle)
{
    if (!isValidTiming(throttle))
    {
        return -1;
    }

    return static_cast<int>(
        setPulseWidth(throttle));
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

    uint32_t sys_clk =
        clock_get_hz(clk_sys);

    uint32_t level =
        width *
        (wrap + 1) /
        (sys_clk / frequency);

    pwm_set_chan_level(
        sliceNumber,
        channel,
        level);

    return level;
}