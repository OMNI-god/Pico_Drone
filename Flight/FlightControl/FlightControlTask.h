#pragma once

#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

#include "Esc.h"
#include "FlightController.h"
#include "AttitudeEstimator.h"
#include "RCPublisher.h"
#include "IMUPublisher.h"
#include "SafetyPublisher.h"

/*
 * Flight control task.
 *
 *     RCPublisher      ->  RC commands
 *     IMUPublisher     ->  accelerometer / gyroscope
 *     SafetyPublisher  ->  arming state
 *     AttitudeEstimator->  roll / pitch / yaw
 *     FlightController ->  PID + mixer
 *     Esc              ->  motor outputs
 */
class FlightControlTask
{
public:
    struct Configuration
    {
        std::uint32_t updateRateHz;

        std::uint32_t stackDepth;

        std::uint32_t priority;

        /*
         * Motor pulse range in microseconds.
         *
         * Standard ESC / servo range:
         *
         *     1000 us = stopped
         *     2000 us = full throttle
         */
        std::uint32_t motorPwmMin;

        std::uint32_t motorPwmMax;

        Configuration()
            : updateRateHz(250),
              stackDepth(2048),
              priority(6),
              motorPwmMin(1000),
              motorPwmMax(2000)
        {
        }
    };

    FlightControlTask(
        RCPublisher &rcPublisher,
        IMUPublisher &imuPublisher,
        SafetyPublisher &safetyPublisher,
        FlightController &flightController,
        AttitudeEstimator &attitudeEstimator,
        Esc **motors,
        std::uint8_t motorCount,
        const Configuration &configuration = Configuration());

    bool start(
        const char *taskName = "FlightControl",
        std::uint32_t stackDepth = 0,
        std::uint32_t priority = 0);

    bool readState(
        FlightControllerState &state) const;

private:
    static void taskEntry(
        void *parameter);

    void run();

    void writeMotors(
        const FlightControllerState &state);

private:
    RCPublisher &_rcPublisher;
    IMUPublisher &_imuPublisher;
    SafetyPublisher &_safetyPublisher;

    FlightController &_flightController;
    AttitudeEstimator &_attitudeEstimator;

    Esc **_motors;
    std::uint8_t _motorCount;

    Configuration _configuration;

    TaskHandle_t _taskHandle = nullptr;
};
