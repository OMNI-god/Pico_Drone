#include "Esc.h"

Esc::Esc(uint32_t pinNumber, uint32_t frequency)
{
    pin = pinNumber;
    this->frequency = frequency;
    initialize() ? initialized = true
                 : throw;
}

bool Esc::initialize()
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    sliceNumber = pwm_gpio_to_slice_num(pin);
    channel = pwm_gpio_to_channel(pin);
    pwm_set_enabled(sliceNumber, true);
    wrap = clock_get_hz(clk_sys);
    wrap /= frequency - 1;
    pwm_set_wrap(sliceNumber, wrap);
    return true;
}

int Esc::setSpeed(int throttle)
{
    uint64_t lvl = 0;
    isValidTiming(throttle) ? lvl = setPulseWidth(throttle)
                            : throw;
    return lvl;
}

uint64_t Esc::setPulseWidth(uint32_t width)
{
    if (!initialize)
        throw;
    uint32_t sys_clk = clock_get_hz(clk_sys);
    uint32_t level = width * (wrap + 1) / (sys_clk / frequency);
    pwm_set_chan_level(sliceNumber, channel, level);
}