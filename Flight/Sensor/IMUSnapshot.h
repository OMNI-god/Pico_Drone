#pragma once

#include <cstdint>

#include "SensorState.h"

struct IMUSnapshot
{
    IMUState state{};

    std::uint32_t sequence = 0;

    bool calibrated = false;
};