#pragma once

#include <cstdint>

/*
 * Simple PID controller.
 *
 * The controller is intentionally small: it is meant to
 * be embedded in the rate / angle control loops of the
 * flight controller.
 *
 *     output = kp * error + ki * integral + kd * derivative
 *
 * The integral term is clamped (anti-windup) and the
 * final output is clamped to +/- outputLimit.
 */
class PID
{
public:
    struct Configuration
    {
        float kp = 1.0f;
        float ki = 0.0f;
        float kd = 0.0f;

        /*
         * Maximum absolute output.
         */
        float outputLimit = 1.0f;

        /*
         * Maximum absolute integral term.
         */
        float integralLimit = 1.0f;
    };

    PID();

    explicit PID(
        const Configuration &config);

    void reset();

    /*
     * Advance the controller.
     *
     * @param setpoint     Desired value.
     * @param measurement  Measured value.
     * @param dt           Time step in seconds.
     *
     * @return Controller output.
     */
    float update(
        float setpoint,
        float measurement,
        float dt);

    void setGains(
        float kp,
        float ki,
        float kd);

    float getLastError() const;

    float getIntegral() const;

private:
    Configuration config_;

    float integral_ = 0.0f;

    float previousError_ = 0.0f;

    bool initialized_ = false;
};
