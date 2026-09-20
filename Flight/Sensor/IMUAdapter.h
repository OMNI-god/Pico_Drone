#pragma once

#include <cstdint>

#include "ICM20948.h"
#include "SensorState.h"

class IMUAdapter
{
public:
    IMUAdapter();

    bool update(
        const ICM20948::SensorData &data,
        uint32_t timestampUs);

    const IMUState &getState() const;

    void invalidate();

private:
    IMUState _state;
};