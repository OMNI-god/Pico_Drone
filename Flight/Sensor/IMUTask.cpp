#include "IMUTask.h"

#include "FreeRTOS.h"
#include "task.h"

IMUTask::IMUTask(
    IMUAdapter &adapter,
    IMUCalibration &calibration,
    IMUPublisher &publisher,
    const Configuration &configuration)
    : adapter_(adapter),
      calibration_(calibration),
      publisher_(publisher),
      configuration_(configuration),
      taskHandle_(nullptr),
      sequence_(0),
      running_(false)
{
}

bool IMUTask::start()
{
    if (taskHandle_ != nullptr)
    {
        return false;
    }

    const BaseType_t result = xTaskCreate(
        &IMUTask::taskEntry,
        "IMU",
        configuration_.taskStackDepth,
        this,
        configuration_.taskPriority,
        &taskHandle_);

    if (result != pdPASS)
    {
        taskHandle_ = nullptr;
        return false;
    }

    return true;
}

bool IMUTask::isRunning() const
{
    return running_;
}

void IMUTask::taskEntry(void *parameter)
{
    auto *task = static_cast<IMUTask *>(parameter);

    task->run();

    task->taskHandle_ = nullptr;
    task->running_ = false;

    vTaskDelete(nullptr);
}

void IMUTask::run()
{
    running_ = true;

    const TickType_t periodTicks =
        pdMS_TO_TICKS(1000 / configuration_.updateRateHz);

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        IMUState sample{};

        if (adapter_.read(sample))
        {
            calibration_.addSample(sample);

            calibration_.apply(sample);

            IMUSnapshot snapshot{};

            snapshot.state = sample;
            snapshot.sequence = ++sequence_;
            snapshot.calibrated = calibration_.isCalibrated();

            publisher_.publish(snapshot);
        }

        vTaskDelayUntil(
            &lastWakeTime,
            periodTicks);
    }
}