#pragma once

#include <array>
#include <cstdint>

constexpr std::uint8_t RC_CHANNEL_COUNT = 16;

// CRSF / ELRS typical calibrated range.
constexpr std::uint16_t RC_DEFAULT_MINIMUM = 172;
constexpr std::uint16_t RC_DEFAULT_CENTER = 992;
constexpr std::uint16_t RC_DEFAULT_MAXIMUM = 1811;

constexpr std::uint16_t RC_DEFAULT_SWITCH_THRESHOLD = 1500;

constexpr float RC_DEFAULT_DEADBAND = 0.02f;

constexpr std::uint32_t RC_DEFAULT_FAILSAFE_TIMEOUT_MS = 100;

enum class RCFunction : std::uint8_t
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

enum class RCFlightMode : std::uint8_t
{
    Angle = 0,
    Horizon = 1,
    Acro = 2
};

struct RCChannelConfig
{
    std::uint16_t minimum = RC_DEFAULT_MINIMUM;
    std::uint16_t center = RC_DEFAULT_CENTER;
    std::uint16_t maximum = RC_DEFAULT_MAXIMUM;

    std::uint16_t switchThreshold =
        RC_DEFAULT_SWITCH_THRESHOLD;

    float deadband = RC_DEFAULT_DEADBAND;

    bool inverted = false;

    RCFunction function = RCFunction::None;
};

struct RCConfig
{
    std::array<RCChannelConfig, RC_CHANNEL_COUNT> channels{};

    std::uint32_t failsafeTimeoutMs =
        RC_DEFAULT_FAILSAFE_TIMEOUT_MS;

    RCConfig();
};