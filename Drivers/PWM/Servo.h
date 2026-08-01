#pragma once

#include "IPwm.h"

class Servo : public IPwm
{
public:
    Servo(uint32_t pinNumber, uint32_t frequency = 50);
    bool initialize() override;
    int setPosition(int position);

private:
    uint64_t setPulseWidth(uint32_t width) override;
};