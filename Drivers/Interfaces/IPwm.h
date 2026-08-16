#pragma once

#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

class IPwm
{
protected:
    uint32_t pin;
    uint32_t sliceNumber;
    uint32_t channel;
    uint32_t wrap;
    uint32_t frequency;
    float clkDiv = 64.0f;
    bool initialized = false;

public:
    virtual ~IPwm() = default;
    virtual bool initialize() = 0;
    virtual uint32_t setPulseWidth(uint32_t pulseWidthUs) = 0;
};