#include "RCInput.h"

#include <algorithm>

RCInput::RCInput(
    const RCConfig &config)
    : _config(config)
{
    _state = RCState{};
    _state.failsafe = true;
}

void RCInput::update(
    const uint16_t *channels,
    uint32_t nowMs)
{
    // --------------------------------------------------------
    // Validate input
    // --------------------------------------------------------

    if (channels == nullptr)
    {
        setFailsafe();
        return;
    }

    // --------------------------------------------------------
    // Validate configuration
    // --------------------------------------------------------

    if (!isValidConfiguration())
    {
        setFailsafe();
        return;
    }

    // --------------------------------------------------------
    // Clear function state
    // --------------------------------------------------------

    _state.roll = 0.0f;
    _state.pitch = 0.0f;
    _state.yaw = 0.0f;
    _state.throttle = 0.0f;

    _state.arm = false;
    _state.beeper = false;
    _state.calibration = false;

    _state.flightMode = 0;

    // --------------------------------------------------------
    // Process all channels
    // --------------------------------------------------------

    for (uint8_t i = 0;
         i < RC_CHANNEL_COUNT;
         ++i)
    {
        const RCChannelConfig &config =
            _config.channels[i];

        const uint16_t rawValue =
            channels[i];

        const float processedValue =
            processChannel(
                rawValue,
                config);

        _state.channels[i] =
            processedValue;

        assignFunction(
            i,
            processedValue,
            rawValue);
    }

    // --------------------------------------------------------
    // Update link state
    // --------------------------------------------------------

    _state.lastUpdateMs = nowMs;
    _state.failsafe = false;
}

float RCInput::processChannel(
    uint16_t value,
    const RCChannelConfig &config) const
{
    switch (config.function)
    {
    case RCFunction::Roll:
    case RCFunction::Pitch:
    case RCFunction::Yaw:
        return normalizeAxis(
            value,
            config);

    case RCFunction::Throttle:
        return normalizeThrottle(
            value,
            config);

    default:
        return normalizeSwitch(
            value,
            config);
    }
}

float RCInput::normalizeAxis(
    uint16_t value,
    const RCChannelConfig &config) const
{
    if (value <= config.minimum)
    {
        return config.inverted
                   ? 1.0f
                   : -1.0f;
    }

    if (value >= config.maximum)
    {
        return config.inverted
                   ? -1.0f
                   : 1.0f;
    }

    float normalized = 0.0f;

    if (value < config.center)
    {
        normalized =
            static_cast<float>(
                static_cast<int32_t>(value) -
                static_cast<int32_t>(config.center)) /
            static_cast<float>(
                static_cast<int32_t>(config.center) -
                static_cast<int32_t>(config.minimum));
    }
    else
    {
        normalized =
            static_cast<float>(
                static_cast<int32_t>(value) -
                static_cast<int32_t>(config.center)) /
            static_cast<float>(
                static_cast<int32_t>(config.maximum) -
                static_cast<int32_t>(config.center));
    }

    normalized =
        std::clamp(
            normalized,
            -1.0f,
            1.0f);

    normalized =
        applyDeadband(
            normalized,
            config.deadband);

    if (config.inverted)
    {
        normalized = -normalized;
    }

    return normalized;
}

float RCInput::normalizeThrottle(
    uint16_t value,
    const RCChannelConfig &config) const
{
    if (value <= config.minimum)
    {
        return config.inverted
                   ? 1.0f
                   : 0.0f;
    }

    if (value >= config.maximum)
    {
        return config.inverted
                   ? 0.0f
                   : 1.0f;
    }

    float normalized =
        static_cast<float>(
            static_cast<int32_t>(value) -
            static_cast<int32_t>(config.minimum)) /
        static_cast<float>(
            static_cast<int32_t>(config.maximum) -
            static_cast<int32_t>(config.minimum));

    normalized =
        std::clamp(
            normalized,
            0.0f,
            1.0f);

    if (config.inverted)
    {
        normalized =
            1.0f - normalized;
    }

    return normalized;
}

float RCInput::normalizeSwitch(
    uint16_t value,
    const RCChannelConfig &config) const
{
    bool active =
        value >= config.switchThreshold;

    if (config.inverted)
    {
        active = !active;
    }

    return active
               ? 1.0f
               : 0.0f;
}

float RCInput::applyDeadband(
    float value,
    float deadband) const
{
    if (deadband <= 0.0f)
    {
        return value;
    }

    if (deadband >= 1.0f)
    {
        return 0.0f;
    }

    if (value >= -deadband &&
        value <= deadband)
    {
        return 0.0f;
    }

    if (value > 0.0f)
    {
        return (value - deadband) /
               (1.0f - deadband);
    }

    return (value + deadband) /
           (1.0f - deadband);
}

void RCInput::assignFunction(
    uint8_t channel,
    float value,
    uint16_t rawValue)
{
    if (!isValidChannelIndex(channel))
    {
        return;
    }

    const RCFunction function =
        _config.channels[channel].function;

    switch (function)
    {
    case RCFunction::Roll:
        _state.roll = value;
        break;

    case RCFunction::Pitch:
        _state.pitch = value;
        break;

    case RCFunction::Throttle:
        _state.throttle = value;
        break;

    case RCFunction::Yaw:
        _state.yaw = value;
        break;

    case RCFunction::Arm:
        _state.arm =
            rawValue >=
            _config.channels[channel]
                .switchThreshold;

        if (_config.channels[channel].inverted)
        {
            _state.arm = !_state.arm;
        }

        break;

    case RCFunction::Beeper:
        _state.beeper =
            rawValue >=
            _config.channels[channel]
                .switchThreshold;

        if (_config.channels[channel].inverted)
        {
            _state.beeper = !_state.beeper;
        }

        break;

    case RCFunction::Calibration:
        _state.calibration =
            rawValue >=
            _config.channels[channel]
                .switchThreshold;

        if (_config.channels[channel].inverted)
        {
            _state.calibration =
                !_state.calibration;
        }

        break;

    case RCFunction::FlightMode:
        _state.flightMode =
            rawValue >=
                    _config.channels[channel]
                        .switchThreshold
                ? 1
                : 0;

        if (_config.channels[channel].inverted)
        {
            _state.flightMode =
                _state.flightMode == 0
                    ? 1
                    : 0;
        }

        break;

    case RCFunction::None:
    case RCFunction::Aux1:
    case RCFunction::Aux2:
    case RCFunction::Aux3:
    case RCFunction::Aux4:
    case RCFunction::Aux5:
    case RCFunction::Aux6:
    case RCFunction::Aux7:
    case RCFunction::Aux8:
        break;
    }
}

void RCInput::updateFailsafe(
    uint32_t nowMs)
{
    if (_state.failsafe)
    {
        return;
    }

    const uint32_t elapsed =
        nowMs - _state.lastUpdateMs;

    if (elapsed >=
        _config.failsafeTimeoutMs)
    {
        setFailsafe();
    }
}

const RCState &
RCInput::getState() const
{
    return _state;
}

bool RCInput::isFailsafe() const
{
    return _state.failsafe;
}

bool RCInput::isValidConfiguration() const
{
    if (_config.failsafeTimeoutMs == 0)
    {
        return false;
    }

    for (uint8_t i = 0;
         i < RC_CHANNEL_COUNT;
         ++i)
    {
        const RCChannelConfig &config =
            _config.channels[i];

        // ----------------------------------------------------
        // Range validation
        // ----------------------------------------------------

        if (config.minimum >=
            config.center)
        {
            return false;
        }

        if (config.center >=
            config.maximum)
        {
            return false;
        }

        // ----------------------------------------------------
        // Deadband validation
        // ----------------------------------------------------

        if (config.deadband < 0.0f ||
            config.deadband >= 1.0f)
        {
            return false;
        }

        // ----------------------------------------------------
        // Switch validation
        // ----------------------------------------------------

        if (config.switchThreshold <
                config.minimum ||
            config.switchThreshold >
                config.maximum)
        {
            return false;
        }

        // ----------------------------------------------------
        // Function-specific validation
        // ----------------------------------------------------

        switch (config.function)
        {
        case RCFunction::Roll:
        case RCFunction::Pitch:
        case RCFunction::Yaw:
        case RCFunction::Throttle:
        case RCFunction::Arm:
        case RCFunction::Beeper:
        case RCFunction::Calibration:
        case RCFunction::FlightMode:
        case RCFunction::None:
        case RCFunction::Aux1:
        case RCFunction::Aux2:
        case RCFunction::Aux3:
        case RCFunction::Aux4:
        case RCFunction::Aux5:
        case RCFunction::Aux6:
        case RCFunction::Aux7:
        case RCFunction::Aux8:
            break;

        default:
            return false;
        }
    }

    return true;
}

bool RCInput::isValidChannelIndex(
    uint8_t index) const
{
    return index < RC_CHANNEL_COUNT;
}

void RCInput::setFailsafe()
{
    _state.failsafe = true;

    _state.arm = false;
    _state.beeper = false;
    _state.calibration = false;

    _state.flightMode = 0;

    _state.roll = 0.0f;
    _state.pitch = 0.0f;
    _state.yaw = 0.0f;
    _state.throttle = 0.0f;

    for (float &channel :
         _state.channels)
    {
        channel = 0.0f;
    }
}