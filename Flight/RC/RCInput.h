#pragma once

#include <cstdint>

#include "RCConfig.h"
#include "RCState.h"

class RCInput
{
public:
    explicit RCInput(
        const RCConfig &config = RCConfig{});

    // Process one complete CRSF channel frame.
    void update(
        const std::uint16_t *channels,
        std::uint64_t nowUs);

    // Check receiver timeout.
    void updateFailsafe(
        std::uint64_t nowUs);

    const RCState &getState() const;

    bool isFailsafe() const;

private:
    float normalizeAxis(
        std::uint16_t value,
        const RCChannelConfig &config) const;

    float normalizeThrottle(
        std::uint16_t value,
        const RCChannelConfig &config) const;

    float normalizeSwitch(
        std::uint16_t value,
        const RCChannelConfig &config) const;

    float applyDeadband(
        float value,
        float deadband) const;

    float processChannel(
        std::uint16_t value,
        const RCChannelConfig &config) const;

    void assignFunction(
        std::uint8_t channel,
        float value,
        std::uint16_t rawValue);

    bool isValidConfiguration() const;

    bool isValidChannelIndex(
        std::uint8_t index) const;

    void setFailsafe();

private:
    RCConfig _config;
    RCState _state;
};