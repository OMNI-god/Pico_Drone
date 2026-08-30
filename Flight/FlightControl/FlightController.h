#pragma once

#include <cstdint>

#include "RCState.h"

struct FlightControlState
{
    bool armed;
    bool failsafe;

    float roll;
    float pitch;
    float yaw;
    float throttle;

    uint8_t flightMode;
};

class FlightController
{
public:
    FlightController();

    void update(
        const RCState &rc,
        uint32_t nowUs);

    const FlightControlState &getState() const;

private:
    FlightControlState _state;

    uint32_t _lastUpdateUs;
    float _dt;
};