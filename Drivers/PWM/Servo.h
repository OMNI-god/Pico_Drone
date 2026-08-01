#pragma once

#include "IPwm.h"

class Servo : public IPwm
{
public:
    Servo(uint32_t pinNumber, uint32_t frequency = 50);

    bool initialize() override;

    int setPosition(int angle);

private:
    uint64_t setPulseWidth(uint32_t pulseWidthUs) override;
};