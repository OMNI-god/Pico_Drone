#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include <cstdint>

class FlightControlTask
{
public:
    static bool create(
        UBaseType_t priority,
        uint16_t stackDepth);

private:
    static void taskEntry(void *parameter);

    static void run();
};