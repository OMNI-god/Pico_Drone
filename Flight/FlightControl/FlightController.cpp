#include "FlightController.h"

#include "FreeRTOS.h"
#include "task.h"

#include <algorithm>
#include <cmath>

// ============================================================================
// Configuration
// ============================================================================

FlightController::Configuration::Configuration()
{
    /*
     * Angle loop:
     *
     * A 1 degree error produces 5 degrees / second.
     * At the maximum angle (45 degrees) the loop
     * requests 225 degrees / second.
     */

    anglePid.kp = 5.0f;
    anglePid.ki = 0.0f;
    anglePid.kd = 0.0f;

    anglePid.outputLimit = maxRateDegPerSec;
    anglePid.integralLimit = 50.0f;

    /*
     * Rate loop:
     *
     * A 100 degrees / second error produces a motor
     * correction of 0.4 (40% of the available
     * correction range).
     */

    ratePid.kp = 0.004f;
    ratePid.ki = 0.0f;
    ratePid.kd = 0.0f;

    ratePid.outputLimit = 1.0f;
    ratePid.integralLimit = 0.2f;

    yawRatePid = ratePid;
}

// ============================================================================
// Construction
// ============================================================================

FlightController::FlightController()
    : FlightController(Configuration{})
{
}

FlightController::FlightController(
    const Configuration &config)
    : config_(config),
      rollAnglePid_(config_.anglePid),
      pitchAnglePid_(config_.anglePid),
      rollRatePid_(config_.ratePid),
      pitchRatePid_(config_.ratePid),
      yawRatePid_(config_.yawRatePid)
{
}

// ============================================================================
// Reset
// ============================================================================

void FlightController::reset()
{
    _state = FlightControllerState{};

    rollAnglePid_.reset();
    pitchAnglePid_.reset();

    rollRatePid_.reset();
    pitchRatePid_.reset();
    yawRatePid_.reset();

    mixer_.reset(_state.motors);
}

// ============================================================================
// Full update
// ============================================================================

void FlightController::update(
    const RCState &rc,
    const IMUSnapshot &imu,
    const AttitudeState &attitude,
    const SafetySnapshot &safety,
    float dt,
    std::uint64_t nowUs)
{
    _state.lastUpdateUs = nowUs;

    _state.imuHealthy =
        imu.state.valid;

    /*
     * ---------------------------------------------------------
     * Failsafe
     * ---------------------------------------------------------
     */

    const bool rcOk =
        rc.valid &&
        !rc.failsafe;

    if (!rcOk ||
        safety.state == SafetyState::Failsafe)
    {
        applyFailsafe();
        return;
    }

    /*
     * ---------------------------------------------------------
     * Commands
     * ---------------------------------------------------------
     */

    _state.roll = rc.roll;
    _state.pitch = rc.pitch;
    _state.yaw = rc.yaw;
    _state.throttle = rc.throttle;

    _state.flightMode =
        rc.flightMode;

    _state.failsafe = false;

    /*
     * ---------------------------------------------------------
     * Arming
     * ---------------------------------------------------------
     */

    const bool armed =
        safety.state == SafetyState::Armed;

    _state.armed = armed;

    if (!armed)
    {
        disarm();
        return;
    }

    /*
     * ---------------------------------------------------------
     * Control
     *
     * A valid attitude estimate is mandatory before
     * any motor output is produced.
     * ---------------------------------------------------------
     */

    if (!attitude.valid ||
        !imu.state.valid)
    {
        disarm();
        return;
    }

    _state.attitudeRoll = attitude.roll;
    _state.attitudePitch = attitude.pitch;
    _state.attitudeYaw = attitude.yaw;

    runControl(
        rc,
        imu,
        attitude,
        dt);
}

// ============================================================================
// RC-only update
// ============================================================================

void FlightController::update(
    const RCState &rc,
    std::uint64_t nowUs)
{
    _state.lastUpdateUs = nowUs;

    if (rc.failsafe || !rc.valid)
    {
        applyFailsafe();
        return;
    }

    _state.roll = rc.roll;
    _state.pitch = rc.pitch;
    _state.yaw = rc.yaw;
    _state.throttle = rc.throttle;

    _state.armed = rc.arm;

    _state.flightMode = rc.flightMode;

    _state.imuHealthy = false;

    disarm();
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

    if (rc.failsafe || !rc.valid)
    {
        applyFailsafe();
        return;
    }

    _state.imuHealthy =
        imu.state.valid;

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

        disarm();
        return;
    }

    _state.roll = rc.roll;
    _state.pitch = rc.pitch;
    _state.yaw = rc.yaw;
    _state.throttle = rc.throttle;

    _state.flightMode =
        rc.flightMode;

    _state.armed = rc.arm;

    _state.failsafe = false;

    disarm();
}

// ============================================================================
// Control
// ============================================================================

void FlightController::runControl(
    const RCState &rc,
    const IMUSnapshot &imu,
    const AttitudeState &attitude,
    float dt)
{
    float targetRollRate = 0.0f;
    float targetPitchRate = 0.0f;

    switch (rc.flightMode)
    {
    // --------------------------------------------------------
    // ANGLE
    //
    // The sticks request an angle. The angle loop
    // converts the angle error into a rate request.
    // --------------------------------------------------------
    case RCFlightMode::Angle:
    {
        const float targetRollAngle =
            rc.roll * config_.maxAngleDeg;

        const float targetPitchAngle =
            rc.pitch * config_.maxAngleDeg;

        targetRollRate =
            rollAnglePid_.update(
                targetRollAngle,
                attitude.roll,
                dt);

        targetPitchRate =
            pitchAnglePid_.update(
                targetPitchAngle,
                attitude.pitch,
                dt);

        break;
    }

    // --------------------------------------------------------
    // HORIZON
    //
    // Small stick deflections request an angle, large
    // deflections request a rate.
    // --------------------------------------------------------
    case RCFlightMode::Horizon:
    {
        const float targetRollAngle =
            rc.roll * config_.maxAngleDeg;

        const float targetPitchAngle =
            rc.pitch * config_.maxAngleDeg;

        const float angleRollRate =
            rollAnglePid_.update(
                targetRollAngle,
                attitude.roll,
                dt);

        const float anglePitchRate =
            pitchAnglePid_.update(
                targetPitchAngle,
                attitude.pitch,
                dt);

        /*
         * Blend factor:
         *
         *     |stick| = 0   ->  angle mode
         *     |stick| = 1   ->  rate mode
         */

        const float rollBlend =
            std::min(
                1.0f,
                std::fabs(rc.roll) * 2.0f);

        const float pitchBlend =
            std::min(
                1.0f,
                std::fabs(rc.pitch) * 2.0f);

        targetRollRate =
            (1.0f - rollBlend) * angleRollRate +
            rollBlend *
                rc.roll *
                config_.maxRateDegPerSec;

        targetPitchRate =
            (1.0f - pitchBlend) * anglePitchRate +
            pitchBlend *
                rc.pitch *
                config_.maxRateDegPerSec;

        break;
    }

    // --------------------------------------------------------
    // ACRO
    //
    // The sticks request a rotation rate.
    // --------------------------------------------------------
    case RCFlightMode::Acro:
    default:
        targetRollRate =
            rc.roll *
            config_.maxRateDegPerSec;

        targetPitchRate =
            rc.pitch *
            config_.maxRateDegPerSec;

        break;
    }

    /*
     * Yaw is always rate controlled.
     */

    const float targetYawRate =
        rc.yaw *
        config_.maxYawRateDegPerSec;

    _state.targetRollRate = targetRollRate;
    _state.targetPitchRate = targetPitchRate;
    _state.targetYawRate = targetYawRate;

    /*
     * Rate loops.
     */

    const float rollCorrection =
        rollRatePid_.update(
            targetRollRate,
            imu.state.gyroX,
            dt);

    const float pitchCorrection =
        pitchRatePid_.update(
            targetPitchRate,
            imu.state.gyroY,
            dt);

    const float yawCorrection =
        yawRatePid_.update(
            targetYawRate,
            imu.state.gyroZ,
            dt);

    /*
     * Mixer.
     */

    mixer_.mix(
        rollCorrection,
        pitchCorrection,
        yawCorrection,
        rc.throttle,
        _state.motors);
}

// ============================================================================
// State access
// ============================================================================

const FlightControllerState &
FlightController::getState() const
{
    return _state;
}

bool FlightController::readState(
    FlightControllerState &state) const
{
    taskENTER_CRITICAL();

    state = _state;

    taskEXIT_CRITICAL();

    return true;
}

// ============================================================================
// Failsafe / disarm
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

    _state.targetRollRate = 0.0f;
    _state.targetPitchRate = 0.0f;
    _state.targetYawRate = 0.0f;

    rollAnglePid_.reset();
    pitchAnglePid_.reset();

    rollRatePid_.reset();
    pitchRatePid_.reset();
    yawRatePid_.reset();

    mixer_.reset(_state.motors);
}

void FlightController::disarm()
{
    _state.throttle = 0.0f;

    _state.targetRollRate = 0.0f;
    _state.targetPitchRate = 0.0f;
    _state.targetYawRate = 0.0f;

    rollAnglePid_.reset();
    pitchAnglePid_.reset();

    rollRatePid_.reset();
    pitchRatePid_.reset();
    yawRatePid_.reset();

    mixer_.reset(_state.motors);
}
