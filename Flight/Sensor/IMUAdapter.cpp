#include "IMUAdapter.h"

IMUAdapter::IMUAdapter()
    : _state{}
{
}

bool IMUAdapter::update(
    const ICM20948::SensorData &data,
    uint32_t timestampUs)
{
    _state.accelX = data.acceleration.x;
    _state.accelY = data.acceleration.y;
    _state.accelZ = data.acceleration.z;

    _state.gyroX = data.gyroscope.x;
    _state.gyroY = data.gyroscope.y;
    _state.gyroZ = data.gyroscope.z;

    _state.timestampUs = timestampUs;

    _state.valid = true;

    return true;
}

const IMUState &
IMUAdapter::getState() const
{
    return _state;
}

void IMUAdapter::invalidate()
{
    _state.valid = false;
}