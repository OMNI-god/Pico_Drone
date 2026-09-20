#pragma once

#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

#include "Elrs.h"
#include "CrsfParser.h"
#include "RCInput.h"
#include "RCPublisher.h"
#include "RCHealth.h"

class RCTask
{
public:
    RCTask(
        Elrs &elrs,
        CrsfParser &parser,
        RCInput &input,
        RCPublisher &publisher);

    bool start(
        const char *taskName = "RC",
        std::uint32_t stackDepth = 1024,
        std::uint32_t priority = 6);

    const RCHealth &getHealth() const;

private:
    static void taskEntry(void *parameter);

    void run();

    void processIncomingData();

    void processFrame(
        std::uint64_t nowUs);

    void updateHealth(
        std::uint64_t nowUs);

private:
    Elrs &_elrs;
    CrsfParser &_parser;
    RCInput &_input;
    RCPublisher &_publisher;

    RCHealth _health{};

    std::uint64_t _lastHealthUpdateUs = 0;
    std::uint32_t _framesAtLastHealthUpdate = 0;

    TaskHandle_t _taskHandle = nullptr;
};