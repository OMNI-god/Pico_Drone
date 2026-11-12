#pragma once

#include <cstdint>

/*
 * Quadcopter X motor mixer.
 *
 * Motor order:
 *
 *     0 = front right
 *     1 = front left
 *     2 = rear right
 *     3 = rear left
 *
 * Control conventions:
 *
 *     roll  > 0  ->  roll right
 *     pitch > 0  ->  pitch forward (nose down)
 *     yaw   > 0  ->  yaw right (clockwise seen from above)
 *
 * Mixing:
 *
 *     M0 (FR) = throttle - roll - pitch - yaw
 *     M1 (FL) = throttle + roll - pitch + yaw
 *     M2 (RR) = throttle - roll + pitch + yaw
 *     M3 (RL) = throttle + roll + pitch - yaw
 *
 * Every output is clamped to the range 0 ... 1.
 */
class MotorMixer
{
public:
    static constexpr std::uint8_t MotorCount = 4;

    void mix(
        float roll,
        float pitch,
        float yaw,
        float throttle,
        float motors[MotorCount]);

    void reset(
        float motors[MotorCount]);

    static float clampMotor(
        float value);
};
