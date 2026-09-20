#include "SafetyTask.h"

#include "pico/time.h"

SafetyTask::SafetyTask(
    RCPublisher &rcPublisher,
    RCTask &rcTask,
    IMUPublisher &imuPublisher,
    SafetyManager &safetyManager,
    SafetyPublisher &safetyPublisher)
    : _rcPublisher(rcPublisher),
      _rcTask(rcTask),
      _imuPublisher(imuPublisher),
      _safetyManager(safetyManager),
      _safetyPublisher(safetyPublisher)
{
}

bool SafetyTask::start(
    const char *taskName,
    std::uint32_t stackDepth,
    std::uint32_t priority)
{
    if (_taskHandle != nullptr)
    {
        return false;
    }

    const BaseType_t result =
        xTaskCreate(
            taskEntry,
            taskName,
            stackDepth,
            this,
            static_cast<UBaseType_t>(priority),
            &_taskHandle);

    return result == pdPASS;
}

void SafetyTask::taskEntry(void *parameter)
{
    auto *task =
        static_cast<SafetyTask *>(parameter);

    if (task == nullptr)
    {
        vTaskDelete(nullptr);
        return;
    }

    task->run();

    vTaskDelete(nullptr);
}

void SafetyTask::run()
{
    /*
     * Safety evaluation runs at 100 Hz.
     *
     * This is intentionally much slower than the IMU task
     * (~1 kHz) but fast enough for the current safety layer.
     */
    constexpr TickType_t TaskPeriod =
        pdMS_TO_TICKS(10);

    TickType_t lastWakeTime =
        xTaskGetTickCount();

    while (true)
    {
        RCState rc{};
        RCHealth rcHealth{};
        IMUSnapshot imu{};

        if (readInputs(rc, rcHealth, imu))
        {
            const std::uint64_t nowUs =
                time_us_64();

            _safetyManager.update(
                rc,
                rcHealth,
                imu,
                nowUs);

            const SafetySnapshot &snapshot =
                _safetyManager.getSnapshot();

            _safetyPublisher.publish(snapshot);

            _snapshot = snapshot;
        }
        else
        {
            /*
             * If an input publisher cannot provide a valid
             * snapshot, we intentionally do NOT invent
             * sensor/RC data here.
             *
             * The next iteration will retry.
             */
        }

        vTaskDelayUntil(
            &lastWakeTime,
            TaskPeriod);
    }
}

bool SafetyTask::readInputs(
    RCState &rc,
    RCHealth &rcHealth,
    IMUSnapshot &imu)
{
    /*
     * RC snapshot
     */
    const bool rcValid =
        _rcPublisher.read(rc);

    /*
     * RCTask currently exposes its health as a const
     * reference. Copy it into our local snapshot so the
     * SafetyManager does not depend on the task's internal
     * object after this function returns.
     */
    rcHealth =
        _rcTask.getHealth();

    /*
     * IMU snapshot
     */
    const bool imuValid =
        _imuPublisher.read(imu);

    return rcValid &&
           imuValid;
}

const SafetySnapshot &SafetyTask::getSnapshot() const
{
    return _snapshot;
}