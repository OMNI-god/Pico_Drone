#pragma once

#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

#include "SafetyManager.h"
#include "SafetyPublisher.h"

#include "RCState.h"
#include "RCHealth.h"
#include "RCPublisher.h"
#include "RCTask.h"

#include "IMUSnapshot.h"
#include "IMUPublisher.h"

class SafetyTask
{
public:
    SafetyTask(
        RCPublisher &rcPublisher,
        RCTask &rcTask,
        IMUPublisher &imuPublisher,
        SafetyManager &safetyManager,
        SafetyPublisher &safetyPublisher);

    bool start(
        const char *taskName = "Safety",
        std::uint32_t stackDepth = 1024,
        std::uint32_t priority = 7);

    const SafetySnapshot &getSnapshot() const;

private:
    static void taskEntry(void *parameter);

    void run();

    bool readInputs(
        RCState &rc,
        RCHealth &rcHealth,
        IMUSnapshot &imu);

private:
    RCPublisher &_rcPublisher;
    RCTask &_rcTask;
    IMUPublisher &_imuPublisher;

    SafetyManager &_safetyManager;
    SafetyPublisher &_safetyPublisher;

    SafetySnapshot _snapshot{};

    TaskHandle_t _taskHandle = nullptr;
};