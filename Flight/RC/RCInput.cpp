#include "RCInput.h"

#include <algorithm>
#include <cmath>

RCInput::RCInput(const RCConfig &config)
    : _config(config),
      _state{}
{
    _state.failsafe = true;
    _state.valid = false;
}

// ============================================================================
// Public API
// ============================================================================

void RCInput::update(
    const std::uint16_t *channels,
    std::uint64_t nowUs)
{
    if (channels == nullptr)
    {
        setFailsafe();
        return;
    }

    if (!isValidConfiguration())
    {
        setFailsafe();
        return;
    }

    // ------------------------------------------------------------
    // Reset function-level state before processing this frame.
    // ------------------------------------------------------------

    _state.roll = 0.0f;
    _state.pitch = 0.0f;
    _state.yaw = 0.0f;
    _state.throttle = 0.0f;

    _state.arm = false;
    _state.beeper = false;
    _state.calibration = false;

    _state.flightMode = RCFlightMode::Angle;

    // ------------------------------------------------------------
    // Process every RC channel.
    // ------------------------------------------------------------

    for (std::uint8_t i = 0;
         i < RC_CHANNEL_COUNT;
         ++i)
    {
        const std::uint16_t rawValue = channels[i];

        _state.rawChannels[i] = rawValue;

        const float processedValue =
            processChannel(
                rawValue,
                _config.channels[i]);

        _state.channels[i] =
            processedValue;

        assignFunction(
            i,
            processedValue,
            rawValue);
    }

    // ------------------------------------------------------------
    // Frame accepted.
    // ------------------------------------------------------------

    _state.valid = true;
    _state.failsafe = false;

    _state.lastUpdateUs = nowUs;

    ++_state.frameCounter;
}

// ============================================================================

void RCInput::updateFailsafe(
    std::uint64_t nowUs)
{
    if (!_state.valid)
    {
        setFailsafe();
        return;
    }

    const std::uint64_t timeoutUs =
        static_cast<std::uint64_t>(
            _config.failsafeTimeoutMs) *
        1000ULL;

    if ((nowUs - _state.lastUpdateUs) >= timeoutUs)
    {
        setFailsafe();
    }
}

// ============================================================================

const RCState &RCInput::getState() const
{
    return _state;
}

// ============================================================================

bool RCInput::isFailsafe() const
{
    return _state.failsafe;
}

// ============================================================================
// Channel processing
// ============================================================================

float RCInput::processChannel(
    std::uint16_t value,
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

// ============================================================================

float RCInput::normalizeAxis(
    std::uint16_t value,
    const RCChannelConfig &config) const
{
    const float minimum =
        static_cast<float>(config.minimum);

    const float center =
        static_cast<float>(config.center);

    const float maximum =
        static_cast<float>(config.maximum);

    const float input =
        static_cast<float>(value);

    float normalized = 0.0f;

    // ------------------------------------------------------------
    // Below center
    // ------------------------------------------------------------

    if (input < center)
    {
        const float range =
            center - minimum;

        if (range > 0.0f)
        {
            normalized =
                (input - center) / range;
        }
    }

    // ------------------------------------------------------------
    // Above center
    // ------------------------------------------------------------

    else
    {
        const float range =
            maximum - center;

        if (range > 0.0f)
        {
            normalized =
                (input - center) / range;
        }
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

// ============================================================================

float RCInput::normalizeThrottle(
    std::uint16_t value,
    const RCChannelConfig &config) const
{
    const float minimum =
        static_cast<float>(config.minimum);

    const float maximum =
        static_cast<float>(config.maximum);

    const float input =
        static_cast<float>(value);

    const float range =
        maximum - minimum;

    if (range <= 0.0f)
    {
        return 0.0f;
    }

    float normalized =
        (input - minimum) / range;

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

// ============================================================================

float RCInput::normalizeSwitch(
    std::uint16_t value,
    const RCChannelConfig &config) const
{
    bool active =
        value >= config.switchThreshold;

    if (config.inverted)
    {
        active = !active;
    }

    return active ? 1.0f : 0.0f;
}

// ============================================================================

float RCInput::applyDeadband(
    float value,
    float deadband) const
{
    if (deadband <= 0.0f)
    {
        return value;
    }

    if (std::fabs(value) <= deadband)
    {
        return 0.0f;
    }

    // Rescale remaining range so that:
    //
    // deadband -> 0
    // +/-1     -> +/-1
    //

    const float sign =
        value >= 0.0f ? 1.0f : -1.0f;

    const float magnitude =
        std::fabs(value);

    const float remaining =
        1.0f - deadband;

    if (remaining <= 0.0f)
    {
        return 0.0f;
    }

    const float scaled =
        (magnitude - deadband) /
        remaining;

    return sign *
           std::clamp(
               scaled,
               0.0f,
               1.0f);
}

// ============================================================================
// Function mapping
// ============================================================================

void RCInput::assignFunction(
    std::uint8_t channel,
    float value,
    std::uint16_t rawValue)
{
    if (!isValidChannelIndex(channel))
    {
        return;
    }

    const RCFunction function =
        _config.channels[channel].function;

    switch (function)
    {
        // --------------------------------------------------------
        // Primary flight controls
        // --------------------------------------------------------

    case RCFunction::Roll:
        _state.roll = value;
        break;

    case RCFunction::Pitch:
        _state.pitch = value;
        break;

    case RCFunction::Yaw:
        _state.yaw = value;
        break;

    case RCFunction::Throttle:
        _state.throttle = value;
        break;

        // --------------------------------------------------------
        // Arm
        // --------------------------------------------------------

    case RCFunction::Arm:
        _state.arm =
            value >= 0.5f;
        break;

        // --------------------------------------------------------
        // Beeper
        // --------------------------------------------------------

    case RCFunction::Beeper:
        _state.beeper =
            value >= 0.5f;
        break;

        // --------------------------------------------------------
        // Calibration
        // --------------------------------------------------------

    case RCFunction::Calibration:
        _state.calibration =
            value >= 0.5f;
        break;

        // --------------------------------------------------------
        // Flight mode
        //
        // Use the raw CRSF value here because flight mode is a
        // three-position switch rather than a binary switch.
        // --------------------------------------------------------

    case RCFunction::FlightMode:

        if (rawValue < 1200)
        {
            _state.flightMode =
                RCFlightMode::Angle;
        }
        else if (rawValue < 1700)
        {
            _state.flightMode =
                RCFlightMode::Horizon;
        }
        else
        {
            _state.flightMode =
                RCFlightMode::Acro;
        }

        break;

        // --------------------------------------------------------
        // AUX channels
        //
        // Currently stored in channels[].
        // Dedicated fields can be added later if needed.
        // --------------------------------------------------------

    case RCFunction::Aux1:
    case RCFunction::Aux2:
    case RCFunction::Aux3:
    case RCFunction::Aux4:
    case RCFunction::Aux5:
    case RCFunction::Aux6:
    case RCFunction::Aux7:
    case RCFunction::Aux8:
    case RCFunction::None:
    default:
        break;
    }
}

// ============================================================================
// Configuration validation
// ============================================================================

bool RCInput::isValidConfiguration() const
{
    if (_config.failsafeTimeoutMs == 0)
    {
        return false;
    }

    for (const auto &channel : _config.channels)
    {
        // --------------------------------------------------------
        // Basic range validation
        // --------------------------------------------------------

        if (!(channel.minimum <
                  channel.center &&
              channel.center <
                  channel.maximum))
        {
            return false;
        }

        // --------------------------------------------------------
        // Deadband validation
        // --------------------------------------------------------

        if (channel.deadband < 0.0f ||
            channel.deadband >= 1.0f)
        {
            return false;
        }

        // --------------------------------------------------------
        // Switch threshold
        // --------------------------------------------------------

        if (channel.switchThreshold <
                channel.minimum ||
            channel.switchThreshold >
                channel.maximum)
        {
            return false;
        }
    }

    return true;
}

// ============================================================================

bool RCInput::isValidChannelIndex(
    std::uint8_t index) const
{
    return index < RC_CHANNEL_COUNT;
}

// ============================================================================
// Failsafe
// ============================================================================

void RCInput::setFailsafe()
{
    _state.valid = false;
    _state.failsafe = true;

    // ------------------------------------------------------------
    // Critical safety outputs.
    // ------------------------------------------------------------

    _state.arm = false;

    _state.throttle = 0.0f;

    _state.roll = 0.0f;
    _state.pitch = 0.0f;
    _state.yaw = 0.0f;

    _state.beeper = false;
    _state.calibration = false;

    _state.flightMode =
        RCFlightMode::Angle;

    // ------------------------------------------------------------
    // Processed channels become safe.
    // ------------------------------------------------------------

    _state.channels.fill(0.0f);
}