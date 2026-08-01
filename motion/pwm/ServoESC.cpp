#include "ServoESC.h"

ServoESC::ServoESC(uint pinNumber, uint frequency)
{
    pin = pinNumber;
    initialized = false;

    gpio_set_function(pin, GPIO_FUNC_PWM);

    slice = pwm_gpio_to_slice_num(pin);
    channel = pwm_gpio_to_channel(pin);

    uint32_t clk = clock_get_hz(clk_sys);

    // Compute wrap for required frequency
    wrap = clk / frequency - 1;
    pwm_set_wrap(slice, wrap);

    pwm_set_enabled(slice, true);
    initialized = true;
}

ServoESC::~ServoESC()
{
    if (initialized) {
        pwm_set_enabled(slice, false);
    }
}

void ServoESC::setThrottle(uint16_t microseconds)
{
    if (!initialized) return;

    // Convert microseconds to PWM level
    // Equivalent to Python: duty = int((throttle / 20000) * 65535)
    uint32_t clk = clock_get_hz(clk_sys);

    uint32_t level = (uint64_t)microseconds * (wrap + 1) / (1000000 / 50);
    pwm_set_chan_level(slice, channel, level);
}

void ServoESC::armESC()
{
    sleep_ms(1000);    // Equivalent to sleep(1)
    setThrottle(900);  // Arm at 900µs
    sleep_ms(3000);    // Equivalent to sleep(3)
}

void ServoESC::disarmESC()
{
    setThrottle(900);  // Minimum throttle
    sleep_ms(5000);    // Equivalent to sleep(5)

    pwm_set_enabled(slice, false);
    initialized = false;
}
