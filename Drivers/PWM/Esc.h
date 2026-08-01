#pragma once

#include "IPwm.h"

class Esc : public IPwm
{
public:
    Esc(uint32_t pinNumber, uint32_t frequency = 50);
    bool initialize() override;
    int setSpeed(int throttle);

private:
    uint64_t setPulseWidth(uint32_t width) override;
};