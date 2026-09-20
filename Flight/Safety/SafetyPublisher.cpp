#include "SafetyPublisher.h"

#include "FreeRTOS.h"
#include "task.h"

SafetyPublisher::SafetyPublisher()
{
    _snapshot = SafetySnapshot{};
    _sequence = 0;
}

void SafetyPublisher::publish(
    const SafetySnapshot &snapshot)
{
    taskENTER_CRITICAL();

    ++_sequence;

    _snapshot = snapshot;

    ++_sequence;

    taskEXIT_CRITICAL();
}

bool SafetyPublisher::read(
    SafetySnapshot &snapshot) const
{
    bool valid = false;

    taskENTER_CRITICAL();

    const std::uint32_t sequenceBefore =
        _sequence;

    snapshot = _snapshot;

    const std::uint32_t sequenceAfter =
        _sequence;

    /*
     * An even sequence means the publisher was not
     * modifying the snapshot during the read.
     */
    valid =
        sequenceBefore == sequenceAfter &&
        (sequenceAfter % 2U) == 0U;

    taskEXIT_CRITICAL();

    return valid;
}