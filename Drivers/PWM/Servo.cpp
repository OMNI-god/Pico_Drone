#include "Servo.h"
#include "algorithm"

Servo::Servo(uint32_t pinNumber, uint32_t frequency)
{
    pin = pinNumber;
    this->frequency = frequency;
    initialize();
}

bool Servo::initialize()
{
    gpio_set_function(pin, GPIO_FUNC_PWM);

    sliceNumber = pwm_gpio_to_slice_num(pin);
    channel = pwm_gpio_to_channel(pin);

    pwm_config config = pwm_get_default_config();

    clkDiv = 64.0f;

    pwm_config_set_clkdiv(&config, clkDiv);

    wrap = clock_get_hz(clk_sys) / (clkDiv * frequency) - 1;

    pwm_config_set_wrap(&config, wrap);

    pwm_init(sliceNumber, &config, true);

    initialized = true;

    return true;
}

int Servo::setPosition(int angle)
{
    if (angle < 0)
        angle = 0;

    if (angle > 180)
        angle = 180;

    uint32_t pulseWidth =
        1000 + ((angle * 1000) / 180);

    return static_cast<int>(setPulseWidth(pulseWidth));
}

uint32_t Servo::setPulseWidth(uint32_t pulseWidthUs)
{
    if (!initialized)
    {
        if (!initialize())
            return 0;
    }

    float pwmClock = clock_get_hz(clk_sys) / clkDiv;

    uint32_t level =
        (pulseWidthUs * pwmClock) / 1000000.0f;

    pwm_set_chan_level(sliceNumber, channel, level);

    return level;
}