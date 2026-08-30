#pragma once

#include <cstdint>

#include "RCConfig.h"
#include "RCState.h"

class RCInput
{
public:
    explicit RCInput(
        const RCConfig &config = RCConfig{});

    void update(
        const uint16_t *channels,
        uint32_t nowMs);

    void updateFailsafe(
        uint32_t nowMs);

    const RCState &getState() const;

    bool isFailsafe() const;

private:
    float normalizeAxis(
        uint16_t value,
        const RCChannelConfig &config) const;

    float normalizeThrottle(
        uint16_t value,
        const RCChannelConfig &config) const;

    float normalizeSwitch(
        uint16_t value,
        const RCChannelConfig &config) const;

    float applyDeadband(
        float value,
        float deadband) const;

    float processChannel(
        uint16_t value,
        const RCChannelConfig &config) const;

    void assignFunction(
        uint8_t channel,
        float value,
        uint16_t rawValue);

    bool isValidConfiguration() const;

    bool isValidChannelIndex(
        uint8_t index) const;

    void setFailsafe();

private:
    RCConfig _config;
    RCState _state;
};