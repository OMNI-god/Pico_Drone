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

    _snapshot = snapshot;

    /*
     * The sequence counter is incremented exactly once per
     * publication. A reader compares the counter before and
     * after copying the snapshot to detect a concurrent write.
     */

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
     * The snapshot is valid only when no publication
     * happened while it was being copied.
     */
    valid =
        sequenceBefore == sequenceAfter;

    taskEXIT_CRITICAL();

    return valid;
}