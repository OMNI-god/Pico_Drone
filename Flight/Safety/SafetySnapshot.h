#pragma once

#include <cstdint>

#include "SafetyState.h"

struct SafetySnapshot
{
    SafetyState state = SafetyState::Disarmed;

    SafetyFault faults = SafetyFault::None;

    bool rcValid = false;
    bool rcFailsafe = true;
    bool rcHealthy = false;

    bool throttleLow = true;
    bool armRequested = false;

    bool imuValid = false;
    bool imuCalibrated = false;
    bool imuDataFresh = false;

    std::uint64_t timestampUs = 0;

    std::uint32_t transitionCounter = 0;
};