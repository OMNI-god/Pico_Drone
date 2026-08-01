#pragma once

#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdio.h"
#include "../../Constants/Constants.h"

class IPwm
{
protected:
    uint32_t pin;
    uint32_t sliceNumber;
    uint32_t channel;
    uint32_t wrap;
    uint32_t frequency;
    bool initialized{false};

    virtual bool isValidTiming(uint32_t input)
    {
        return input < PWM_MIN || input > PWM_MAX;
    }

public:
    virtual ~IPwm() = default;
    virtual bool initialize() = 0;
    virtual uint64_t setPulseWidth(uint32_t width) = 0;
};