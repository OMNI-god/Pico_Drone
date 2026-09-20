#include "IMUPublisher.h"

#include "FreeRTOS.h"
#include "task.h"

IMUPublisher::IMUPublisher()
    : snapshot_{},
      sequence_(0)
{
}

void IMUPublisher::publish(const IMUSnapshot &snapshot)
{
    taskENTER_CRITICAL();

    snapshot_ = snapshot;
    sequence_ = snapshot.sequence;

    taskEXIT_CRITICAL();
}

bool IMUPublisher::read(IMUSnapshot &snapshot) const
{
    taskENTER_CRITICAL();

    snapshot = snapshot_;

    taskEXIT_CRITICAL();

    return snapshot.state.valid;
}