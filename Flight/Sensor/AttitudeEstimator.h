#pragma once

#include <cstdint>

#include "SensorState.h"

struct AttitudeState
{
    // Unit: degrees

    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    bool valid = false;

    uint32_t timestampUs = 0;
};

class AttitudeEstimator
{
public:
    AttitudeEstimator();

    void reset();

    bool update(
        const IMUState &imu,
        float dt);

    const AttitudeState &getState() const;

private:
    float calculateRollFromAccelerometer(
        const IMUState &imu) const;

    float calculatePitchFromAccelerometer(
        const IMUState &imu) const;

    AttitudeState _state;

    bool _initialized;
};