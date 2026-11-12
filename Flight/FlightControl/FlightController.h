#pragma once

#include <cstdint>

#include "RCState.h"
#include "SensorState.h"
#include "IMUSnapshot.h"
#include "AttitudeEstimator.h"
#include "SafetySnapshot.h"
#include "PID.h"
#include "MotorMixer.h"

struct FlightControllerState
{
    // ------------------------------------------------------------
    // RC commands
    //
    // Axis:
    //   Roll/Pitch/Yaw = -1.0 ... +1.0
    //
    // Throttle:
    //   0.0 ... 1.0
    // ------------------------------------------------------------

    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float throttle = 0.0f;

    // ------------------------------------------------------------
    // Flight state
    // ------------------------------------------------------------

    bool armed = false;
    bool failsafe = true;

    RCFlightMode flightMode =
        RCFlightMode::Angle;

    // ------------------------------------------------------------
    // Sensor state
    // ------------------------------------------------------------

    bool imuHealthy = false;

    // ------------------------------------------------------------
    // Attitude estimate (degrees)
    // ------------------------------------------------------------

    float attitudeRoll = 0.0f;
    float attitudePitch = 0.0f;
    float attitudeYaw = 0.0f;

    // ------------------------------------------------------------
    // Control targets (degrees / second)
    // ------------------------------------------------------------

    float targetRollRate = 0.0f;
    float targetPitchRate = 0.0f;
    float targetYawRate = 0.0f;

    // ------------------------------------------------------------
    // Motor outputs
    //
    // Range:
    //   0.0 ... 1.0
    //
    // Motor order:
    //   0 = front right
    //   1 = front left
    //   2 = rear right
    //   3 = rear left
    // ------------------------------------------------------------

    float motors[MotorMixer::MotorCount] = {
        0.0f,
        0.0f,
        0.0f,
        0.0f};

    // ------------------------------------------------------------
    // Timing
    // ------------------------------------------------------------

    std::uint64_t lastUpdateUs = 0;
};

class FlightController
{
public:
    struct Configuration
    {
        /*
         * Maximum angle requested in ANGLE / HORIZON
         * mode, in degrees.
         */
        float maxAngleDeg = 45.0f;

        /*
         * Maximum rotation rate requested in ACRO
         * mode, in degrees per second.
         */
        float maxRateDegPerSec = 360.0f;

        /*
         * Maximum yaw rate, in degrees per second.
         */
        float maxYawRateDegPerSec = 180.0f;

        /*
         * Angle (outer) loop.
         *
         * Input:  angle error in degrees
         * Output: desired rate in degrees / second
         */
        PID::Configuration anglePid;

        /*
         * Rate (inner) loop.
         *
         * Input:  rate error in degrees / second
         * Output: motor correction in -1 ... +1
         */
        PID::Configuration ratePid;

        /*
         * Yaw rate loop.
         */
        PID::Configuration yawRatePid;

        Configuration();
    };

    FlightController();

    explicit FlightController(
        const Configuration &config);

    /*
     * Reset the controller and every PID loop.
     */
    void reset();

    /*
     * Full update:
     *
     *     RC + IMU + attitude + safety -> motors
     */
    void update(
        const RCState &rc,
        const IMUSnapshot &imu,
        const AttitudeState &attitude,
        const SafetySnapshot &safety,
        float dt,
        std::uint64_t nowUs);

    /*
     * RC-only update.
     *
     * Used while validating the RC pipeline.
     */
    void update(
        const RCState &rc,
        std::uint64_t nowUs);

    /*
     * RC + IMU update without attitude estimation.
     */
    void update(
        const RCState &rc,
        const IMUSnapshot &imu,
        std::uint64_t nowUs);

    const FlightControllerState &getState() const;

    /*
     * Thread-safe state copy.
     */
    bool readState(
        FlightControllerState &state) const;

private:
    void applyFailsafe();

    void disarm();

    void runControl(
        const RCState &rc,
        const IMUSnapshot &imu,
        const AttitudeState &attitude,
        float dt);

private:
    Configuration config_;

    FlightControllerState _state;

    PID rollAnglePid_;
    PID pitchAnglePid_;

    PID rollRatePid_;
    PID pitchRatePid_;
    PID yawRatePid_;

    MotorMixer mixer_;
};
