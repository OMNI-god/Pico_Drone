#include "MotorMixer.h"

#include <algorithm>
#include <cmath>

float MotorMixer::clampMotor(
    float value)
{
    if (!std::isfinite(value))
    {
        return 0.0f;
    }

    return std::clamp(
        value,
        0.0f,
        1.0f);
}

void MotorMixer::mix(
    float roll,
    float pitch,
    float yaw,
    float throttle,
    float motors[MotorCount])
{
    if (motors == nullptr)
    {
        return;
    }

    const float clampedThrottle =
        clampMotor(throttle);

    motors[0] =
        clampMotor(
            clampedThrottle - roll - pitch - yaw);

    motors[1] =
        clampMotor(
            clampedThrottle + roll - pitch + yaw);

    motors[2] =
        clampMotor(
            clampedThrottle - roll + pitch + yaw);

    motors[3] =
        clampMotor(
            clampedThrottle + roll + pitch - yaw);
}

void MotorMixer::reset(
    float motors[MotorCount])
{
    if (motors == nullptr)
    {
        return;
    }

    for (std::uint8_t i = 0;
         i < MotorCount;
         ++i)
    {
        motors[i] = 0.0f;
    }
}
