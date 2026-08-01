#pragma once
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

class ServoESC {
private:
    uint pin;
    uint slice;
    uint channel;
    bool initialized;
    uint32_t wrap;

public:
    ServoESC(uint pinNumber, uint frequency = 50);
    ~ServoESC();

    void setThrottle(uint16_t microseconds);
    void armESC();
    void disarmESC();
};
