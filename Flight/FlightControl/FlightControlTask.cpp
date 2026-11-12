#include "FlightControlTask.h"

#include "pico/stdlib.h"
#include "pico/time.h"

// ============================================================
// Constructor
// ============================================================

FlightControlTask::FlightControlTask(
    RCPublisher &rcPublisher,
    IMUPublisher &imuPublisher,
    SafetyPublisher &safetyPublisher,
    FlightController &flightController,
    AttitudeEstimator &attitudeEstimator,
    Esc **motors,
    std::uint8_t motorCount,
    const Configuration &configuration)
    : _rcPublisher(rcPublisher),
      _imuPublisher(imuPublisher),
      _safetyPublisher(safetyPublisher),
      _flightController(flightController),
      _attitudeEstimator(attitudeEstimator),
      _motors(motors),
      _motorCount(motorCount),
      _configuration(configuration)
{
}

// ============================================================
// Start
// ============================================================

bool FlightControlTask::start(
    const char *taskName,
    std::uint32_t stackDepth,
    std::uint32_t priority)
{
    if (_taskHandle != nullptr)
    {
        return false;
    }

    const std::uint32_t actualStackDepth =
        stackDepth == 0 ?
            _configuration.stackDepth :
            stackDepth;

    const std::uint32_t actualPriority =
        priority == 0 ?
            _configuration.priority :
            priority;

    const BaseType_t result =
        xTaskCreate(
            taskEntry,
            taskName,
            actualStackDepth,
            this,
            static_cast<UBaseType_t>(actualPriority),
            &_taskHandle);

    return result == pdPASS;
}

// ============================================================
// Task entry
// ============================================================

void FlightControlTask::taskEntry(
    void *parameter)
{
    auto *task =
        static_cast<FlightControlTask *>(parameter);

    if (task == nullptr)
    {
        vTaskDelete(nullptr);
        return;
    }

    task->run();

    vTaskDelete(nullptr);
}

// ============================================================
// Main loop
// ============================================================

void FlightControlTask::run()
{
    const TickType_t periodTicks =
        pdMS_TO_TICKS(
            1000 / _configuration.updateRateHz);

    TickType_t lastWakeTime =
        xTaskGetTickCount();

    std::uint64_t lastTimeUs =
        time_us_64();

    while (true)
    {
        RCState rc{};
        IMUSnapshot imu{};
        SafetySnapshot safety{};

        /*
         * Inputs.
         *
         * A publisher that has no valid data yet is
         * reported as invalid; the flight controller
         * will apply failsafe in that case.
         */

        (void)_rcPublisher.read(rc);
        (void)_imuPublisher.read(imu);
        (void)_safetyPublisher.read(safety);

        /*
         * Timing.
         */

        const std::uint64_t nowUs =
            time_us_64();

        float dt =
            static_cast<float>(
                nowUs - lastTimeUs) /
            1000000.0f;

        lastTimeUs = nowUs;

        if (!(dt > 0.0f))
        {
            dt =
                1.0f /
                static_cast<float>(
                    _configuration.updateRateHz);
        }

        if (dt > 0.05f)
        {
            dt = 0.05f;
        }

        /*
         * Attitude estimation.
         */

        _attitudeEstimator.update(
            imu.state,
            dt);

        const AttitudeState &attitude =
            _attitudeEstimator.getState();

        /*
         * Flight controller.
         */

        _flightController.update(
            rc,
            imu,
            attitude,
            safety,
            dt,
            nowUs);

        FlightControllerState state{};

        _flightController.readState(state);

        writeMotors(state);

        vTaskDelayUntil(
            &lastWakeTime,
            periodTicks);
    }
}

// ============================================================
// Motor outputs
// ============================================================

void FlightControlTask::writeMotors(
    const FlightControllerState &state)
{
    if (_motors == nullptr)
    {
        return;
    }

    const std::uint32_t motorRange =
        _configuration.motorPwmMax -
        _configuration.motorPwmMin;

    for (std::uint8_t i = 0;
         i < _motorCount &&
         i < MotorMixer::MotorCount;
         ++i)
    {
        if (_motors[i] == nullptr)
        {
            continue;
        }

        std::uint32_t pulseWidth =
            _configuration.motorPwmMin;

        if (state.armed)
        {
            const float motor =
                state.motors[i];

            pulseWidth +=
                static_cast<std::uint32_t>(
                    motor *
                    static_cast<float>(
                        motorRange));
        }

        _motors[i]->setSpeed(
            static_cast<int>(pulseWidth));
    }
}

// ============================================================
// State access
// ============================================================

bool FlightControlTask::readState(
    FlightControllerState &state) const
{
    return _flightController.readState(state);
}
