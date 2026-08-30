#pragma once

#include <cstdint>

#include "RCConfig.h"

struct RCState
{
    // --------------------------------------------------------
    // Processed channels
    //
    // Axis:
    //      -1.0 ... +1.0
    //
    // Throttle:
    //       0.0 ... +1.0
    //
    // Switch:
    //       0.0 ... 1.0
    // --------------------------------------------------------

    float channels[RC_CHANNEL_COUNT]{};

    // --------------------------------------------------------
    // Primary flight controls
    // --------------------------------------------------------

    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float throttle = 0.0f;

    // --------------------------------------------------------
    // Functions
    // --------------------------------------------------------

    bool arm = false;
    bool beeper = false;
    bool calibration = false;

    uint8_t flightMode = 0;

    // --------------------------------------------------------
    // Link state
    // --------------------------------------------------------

    bool failsafe = true;

    uint32_t lastUpdateMs = 0;
};