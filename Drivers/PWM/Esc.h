#pragma once

#include <cstdint>

#include "IPwm.h"

class Esc : public IPwm
{
public:
    Esc(
        uint32_t pinNumber,
        uint32_t frequency = 50);

    bool initialize() override;

    /*
     * setSpeed() takes the pulse width in microseconds
     * (1000 ... 2000 us for a standard ESC).
     *
     * The value is validated and then forwarded to
     * setPulseWidth().
     */
    int setSpeed(
        int pulseWidthUs);

private:
    uint32_t setPulseWidth(
        uint32_t width) override;

    bool isValidTiming(
        uint32_t pulseWidthUs);
};