#pragma once

#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

#include "IMUAdapter.h"
#include "IMUCalibration.h"
#include "IMUPublisher.h"

class IMUTask
{
public:
    struct Configuration
    {
        std::uint32_t updateRateHz;
        std::uint16_t taskStackDepth;
        std::uint32_t taskPriority;

        Configuration()
            : updateRateHz(1000),
              taskStackDepth(1024),
              taskPriority(5)
        {
        }
    };

    IMUTask(
        IMUAdapter &adapter,
        IMUCalibration &calibration,
        IMUPublisher &publisher,
        const Configuration &configuration = Configuration());

    bool start();

    bool isRunning() const;

    /*
     * Request an IMU calibration reset.
     *
     * The reset is executed by this task on its next
     * cycle, so the calibration object is only ever
     * accessed from the IMU task itself.
     */
    void requestCalibrationReset();

private:
    static void taskEntry(void *parameter);

    void run();

    IMUAdapter &adapter_;
    IMUCalibration &calibration_;
    IMUPublisher &publisher_;

    Configuration configuration_;

    TaskHandle_t taskHandle_;

    std::uint32_t sequence_;
    bool running_;

    volatile bool calibrationResetRequested_;
};