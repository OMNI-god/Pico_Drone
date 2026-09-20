#include "RCPublisher.h"

#include "FreeRTOS.h"
#include "task.h"

RCPublisher::RCPublisher()
    : _state{},
      _sequence(0)
{
}

// ============================================================================

void RCPublisher::publish(
    const RCState &state)
{
    taskENTER_CRITICAL();

    _state = state;

    ++_sequence;

    taskEXIT_CRITICAL();
}

// ============================================================================

bool RCPublisher::read(
    RCState &state) const
{
    taskENTER_CRITICAL();

    state = _state;

    taskEXIT_CRITICAL();

    return state.valid;
}