#pragma once

#include <cstdint>

#include "SensorState.h"
#include "ICM20948.h"

class IMUAdapter
{
public:
    explicit IMUAdapter(ICM20948 &imu);

    bool read(IMUState &state);

private:
    ICM20948 &imu_;
};