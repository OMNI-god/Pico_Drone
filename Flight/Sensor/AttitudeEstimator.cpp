#include "AttitudeEstimator.h"

#include <cmath>

namespace
{
    constexpr float RAD_TO_DEG =
        57.29577951308232f;

    // Complementary filter.
    //
    // Higher value:
    //     trust gyro more
    //
    // Lower value:
    //     trust accelerometer more
    //
    constexpr float FILTER_ALPHA = 0.98f;

    constexpr float MIN_DT = 0.0001f;
    constexpr float MAX_DT = 0.02f;
}

AttitudeEstimator::AttitudeEstimator()
    : _state{},
      _initialized(false)
{
}

void AttitudeEstimator::reset()
{
    _state = AttitudeState{};
    _initialized = false;
}

bool AttitudeEstimator::update(
    const IMUState &imu,
    float dt)
{
    if (!imu.valid)
    {
        _state.valid = false;
        return false;
    }

    if (dt < MIN_DT ||
        dt > MAX_DT)
    {
        _state.valid = false;
        return false;
    }

    const float accelRoll =
        calculateRollFromAccelerometer(imu);

    const float accelPitch =
        calculatePitchFromAccelerometer(imu);

    // --------------------------------------------------------
    // First valid sample
    // --------------------------------------------------------

    if (!_initialized)
    {
        _state.roll = accelRoll;
        _state.pitch = accelPitch;
        _state.yaw = 0.0f;

        _state.valid = true;

        _initialized = true;

        return true;
    }

    // --------------------------------------------------------
    // Gyroscope integration
    //
    // gyro units:
    // degrees / second
    //
    // dt:
    // seconds
    //
    // result:
    // degrees
    // --------------------------------------------------------

    const float gyroRoll =
        _state.roll +
        imu.gyroX * dt;

    const float gyroPitch =
        _state.pitch +
        imu.gyroY * dt;

    const float gyroYaw =
        _state.yaw +
        imu.gyroZ * dt;

    // --------------------------------------------------------
    // Complementary filter
    // --------------------------------------------------------

    _state.roll =
        FILTER_ALPHA * gyroRoll +
        (1.0f - FILTER_ALPHA) * accelRoll;

    _state.pitch =
        FILTER_ALPHA * gyroPitch +
        (1.0f - FILTER_ALPHA) * accelPitch;

    // --------------------------------------------------------
    // Yaw
    //
    // No absolute yaw reference yet.
    //
    // GY271 will eventually correct this.
    // --------------------------------------------------------

    _state.yaw = gyroYaw;

    _state.valid = true;

    _state.timestampUs = imu.timestampUs;

    return true;
}

float AttitudeEstimator::calculateRollFromAccelerometer(
    const IMUState &imu) const
{
    return std::atan2(
               imu.accelY,
               imu.accelZ) *
           RAD_TO_DEG;
}

float AttitudeEstimator::calculatePitchFromAccelerometer(
    const IMUState &imu) const
{
    return std::atan2(
               -imu.accelX,
               std::sqrt(
                   imu.accelY * imu.accelY +
                   imu.accelZ * imu.accelZ)) *
           RAD_TO_DEG;
}

const AttitudeState &
AttitudeEstimator::getState() const
{
    return _state;
}