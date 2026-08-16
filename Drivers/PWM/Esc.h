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

    int setSpeed(
        int throttle);

private:
    uint32_t setPulseWidth(
        uint32_t width) override;

    bool isValidTiming(
        uint32_t pulseWidthUs);
};