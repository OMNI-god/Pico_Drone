#include "IMUAdapter.h"

#include "pico/stdlib.h"

IMUAdapter::IMUAdapter(ICM20948 &imu)
    : imu_(imu)
{
}

bool IMUAdapter::read(IMUState &state)
{
    ICM20948::SensorData sensorData{};

    if (!imu_.read(sensorData))
    {
        state.valid = false;
        return false;
    }

    state.accelX = sensorData.acceleration.x;
    state.accelY = sensorData.acceleration.y;
    state.accelZ = sensorData.acceleration.z;

    state.gyroX = sensorData.gyroscope.x;
    state.gyroY = sensorData.gyroscope.y;
    state.gyroZ = sensorData.gyroscope.z;

    // Pico SDK absolute time in microseconds.
    state.timestampUs = time_us_64();

    state.valid = true;

    return true;
}