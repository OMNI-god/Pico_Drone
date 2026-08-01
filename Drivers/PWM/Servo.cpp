#include "Servo.h"

Servo::Servo(uint32_t pinNumber, uint32_t frequency)
{
    pin = pinNumber;
    this->frequency = frequency;
    initialized = initialize();
}

bool Servo::initialize()
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    sliceNumber = pwm_gpio_to_slice_num(pin);
    channel = pwm_gpio_to_channel(pin);

    pwm_set_enabled(sliceNumber, true);

    wrap = clock_get_hz(clk_sys) / (frequency - 1);
    pwm_set_wrap(sliceNumber, wrap);
    return true;
}

int Servo::setPosition(int position)
{
    if (!isValidTiming(static_cast<uint32_t>(position)))
        return 0;

    return static_cast<int>(setPulseWidth(static_cast<uint32_t>(position)));
}

uint64_t Servo::setPulseWidth(uint32_t width)
{
    if (!initialized)
    {
        initialized = initialize();
        if (!initialized)
            return 0;
    }

    uint32_t sys_clk = clock_get_hz(clk_sys);
    uint32_t level = width * (wrap + 1) / (sys_clk / frequency);
    pwm_set_chan_level(sliceNumber, channel, level);
    return level;
}