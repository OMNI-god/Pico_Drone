#include "FlightController.h"

FlightController::FlightController()
    : _state{},
      _lastUpdateUs(0),
      _dt(0.0f)
{
}

void FlightController::update(
    const RCState &rc,
    uint32_t nowUs)
{
    // --------------------------------------------------------
    // Calculate loop delta time
    // --------------------------------------------------------

    if (_lastUpdateUs == 0)
    {
        _dt = 0.0f;
    }
    else
    {
        const uint32_t elapsedUs =
            nowUs - _lastUpdateUs;

        _dt =
            static_cast<float>(elapsedUs) *
            0.000001f;
    }

    _lastUpdateUs = nowUs;

    // --------------------------------------------------------
    // Copy RC state
    // --------------------------------------------------------

    _state.failsafe = rc.failsafe;

    _state.roll = rc.roll;
    _state.pitch = rc.pitch;
    _state.yaw = rc.yaw;
    _state.throttle = rc.throttle;

    _state.flightMode = rc.flightMode;

    // --------------------------------------------------------
    // Safety
    // --------------------------------------------------------

    _state.armed =
        rc.arm &&
        !rc.failsafe;
}

const FlightControlState &
FlightController::getState() const
{
    return _state;
}