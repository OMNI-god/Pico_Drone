#pragma once

#include <array>
#include <cstdint>

// ------------------------------------------------------------
// RC system constants
// ------------------------------------------------------------

constexpr uint8_t RC_CHANNEL_COUNT = 16;

// Standard CRSF channel range
constexpr uint16_t RC_DEFAULT_MINIMUM = 172;
constexpr uint16_t RC_DEFAULT_CENTER = 992;
constexpr uint16_t RC_DEFAULT_MAXIMUM = 1811;

// Default switch threshold
constexpr uint16_t RC_DEFAULT_SWITCH_THRESHOLD = 1500;

// Default deadband
constexpr float RC_DEFAULT_DEADBAND = 0.02f;

// Default failsafe timeout
constexpr uint32_t RC_DEFAULT_FAILSAFE_TIMEOUT_MS = 100;

// ------------------------------------------------------------
// RC channel functions
// ------------------------------------------------------------

enum class RCFunction : uint8_t
{
    None = 0,

    Roll,
    Pitch,
    Throttle,
    Yaw,

    Arm,
    Beeper,
    Calibration,
    FlightMode,

    Aux1,
    Aux2,
    Aux3,
    Aux4,
    Aux5,
    Aux6,
    Aux7,
    Aux8
};

// ------------------------------------------------------------
// Individual channel configuration
// ------------------------------------------------------------

struct RCChannelConfig
{
    uint16_t minimum =
        RC_DEFAULT_MINIMUM;

    uint16_t center =
        RC_DEFAULT_CENTER;

    uint16_t maximum =
        RC_DEFAULT_MAXIMUM;

    uint16_t switchThreshold =
        RC_DEFAULT_SWITCH_THRESHOLD;

    float deadband =
        RC_DEFAULT_DEADBAND;

    bool inverted = false;

    RCFunction function =
        RCFunction::None;
};

// ------------------------------------------------------------
// Complete RC configuration
// ------------------------------------------------------------

struct RCConfig
{
    std::array<
        RCChannelConfig,
        RC_CHANNEL_COUNT>
        channels{};

    uint32_t failsafeTimeoutMs =
        RC_DEFAULT_FAILSAFE_TIMEOUT_MS;

    RCConfig();
};