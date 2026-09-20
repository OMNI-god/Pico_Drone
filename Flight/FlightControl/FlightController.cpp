#include "FlightController.h"

FlightController::FlightController()
    : _state{}
{
}

// ============================================================================
// RC-only update
// ============================================================================
//
// This is useful while we are validating the RC pipeline.
//
// It does NOT perform PID, attitude control, or motor mixing yet.
//

void FlightController::update(
    const RCState &rc,
    std::uint64_t nowUs)
{
    _state.lastUpdateUs = nowUs;

    // ------------------------------------------------------------
    // Failsafe
    // ------------------------------------------------------------

    if (rc.failsafe || !rc.valid)
    {
        applyFailsafe();
        return;
    }

    // ------------------------------------------------------------
    // Copy RC commands
    // ------------------------------------------------------------

    _state.roll = rc.roll;
    _state.pitch = rc.pitch;
    _state.yaw = rc.yaw;
    _state.throttle = rc.throttle;

    // ------------------------------------------------------------
    // Arming
    //
    // For now the flight controller mirrors the RC arm state.
    //
    // Later this must become an ArmingManager with:
    //
    //   - throttle-low check
    //   - sensor-health check
    //   - calibration state
    //   - failsafe state
    //   - startup delay
    //   - disarm conditions
    // ------------------------------------------------------------

    _state.armed = rc.arm;

    // ------------------------------------------------------------
    // Flight mode
    // ------------------------------------------------------------

    _state.flightMode = rc.flightMode;

    // ------------------------------------------------------------
    // No IMU in this overload.
    // ------------------------------------------------------------

    _state.imuHealthy = false;
}

// ============================================================================
// RC + IMU update
// ============================================================================

void FlightController::update(
    const RCState &rc,
    const IMUSnapshot &imu,
    std::uint64_t nowUs)
{
    _state.lastUpdateUs = nowUs;

    // ------------------------------------------------------------
    // RC failsafe
    // ------------------------------------------------------------

    if (rc.failsafe || !rc.valid)
    {
        applyFailsafe();
        return;
    }

    // ------------------------------------------------------------
    // IMU validity
    // ------------------------------------------------------------

    _state.imuHealthy =
        imu.state.valid;

    // ------------------------------------------------------------
    // For now we require a valid IMU before arming.
    //
    // This is only the foundation.
    // A dedicated ArmingManager will eventually own this logic.
    // ------------------------------------------------------------

    if (!imu.state.valid)
    {
        _state.roll = 0.0f;
        _state.pitch = 0.0f;
        _state.yaw = 0.0f;
        _state.throttle = 0.0f;

        _state.armed = false;

        _state.flightMode =
            RCFlightMode::Angle;

        _state.failsafe = true;

        return;
    }

    // ------------------------------------------------------------
    // RC commands
    // ------------------------------------------------------------

    _state.roll = rc.roll;
    _state.pitch = rc.pitch;
    _state.yaw = rc.yaw;
    _state.throttle = rc.throttle;

    // ------------------------------------------------------------
    // Flight mode
    // ------------------------------------------------------------

    _state.flightMode =
        rc.flightMode;

    // ------------------------------------------------------------
    // Arm
    // ------------------------------------------------------------

    _state.armed =
        rc.arm;

    _state.failsafe = false;
}

// ============================================================================

const FlightControllerState &
FlightController::getState() const
{
    return _state;
}

// ============================================================================

void FlightController::applyFailsafe()
{
    _state.roll = 0.0f;
    _state.pitch = 0.0f;
    _state.yaw = 0.0f;
    _state.throttle = 0.0f;

    _state.armed = false;

    _state.flightMode =
        RCFlightMode::Angle;

    _state.failsafe = true;
}