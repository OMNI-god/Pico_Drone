#pragma once

#include <array>
#include <cstdint>

#include "RCConfig.h"

struct RCState
{
    // ------------------------------------------------------------
    // Raw receiver values
    // ------------------------------------------------------------

    std::array<std::uint16_t, RC_CHANNEL_COUNT> rawChannels{};

    // ------------------------------------------------------------
    // Processed channels
    //
    // Axis:
    //   Roll/Pitch/Yaw = -1.0 ... +1.0
    //
    // Throttle:
    //   0.0 ... 1.0
    //
    // Switch:
    //   0.0 or 1.0
    // ------------------------------------------------------------

    std::array<float, RC_CHANNEL_COUNT> channels{};

    // ------------------------------------------------------------
    // Primary flight controls
    // ------------------------------------------------------------

    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float throttle = 0.0f;

    // ------------------------------------------------------------
    // Flight switches
    // ------------------------------------------------------------

    bool arm = false;
    bool beeper = false;
    bool calibration = false;

    RCFlightMode flightMode =
        RCFlightMode::Angle;

    // ------------------------------------------------------------
    // Link state
    // ------------------------------------------------------------

    bool valid = false;
    bool failsafe = true;

    // Number of successfully processed RC frames.
    std::uint32_t frameCounter = 0;

    // Timestamp of the most recently accepted frame.
    std::uint64_t lastUpdateUs = 0;
};