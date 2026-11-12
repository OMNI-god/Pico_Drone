#include "PID.h"

#include <algorithm>
#include <cmath>

PID::PID()
    : PID(Configuration{})
{
}

PID::PID(
    const Configuration &config)
    : config_(config)
{
}

void PID::reset()
{
    integral_ = 0.0f;

    previousError_ = 0.0f;

    initialized_ = false;
}

float PID::update(
    float setpoint,
    float measurement,
    float dt)
{
    /*
     * A non-positive time step would corrupt the
     * integral and derivative terms.
     */

    if (!(dt > 0.0f) ||
        !std::isfinite(dt))
    {
        return 0.0f;
    }

    const float error =
        setpoint - measurement;

    /*
     * Integral term with clamping.
     */

    integral_ += error * dt;

    integral_ =
        std::clamp(
            integral_,
            -config_.integralLimit,
            config_.integralLimit);

    /*
     * Derivative term.
     *
     * The first call after reset has no previous
     * error, so the derivative term is skipped.
     */

    float derivative = 0.0f;

    if (initialized_)
    {
        derivative =
            (error - previousError_) / dt;
    }

    previousError_ = error;

    initialized_ = true;

    /*
     * Combine and clamp.
     */

    float output =
        config_.kp * error +
        config_.ki * integral_ +
        config_.kd * derivative;

    output =
        std::clamp(
            output,
            -config_.outputLimit,
            config_.outputLimit);

    return output;
}

void PID::setGains(
    float kp,
    float ki,
    float kd)
{
    config_.kp = kp;
    config_.ki = ki;
    config_.kd = kd;
}

float PID::getLastError() const
{
    return previousError_;
}

float PID::getIntegral() const
{
    return integral_;
}
