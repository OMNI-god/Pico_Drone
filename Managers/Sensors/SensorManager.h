#pragma once

#include "ICM20948.h"
#include "GY271.h"
#include "BMP280.h"
#include "ADXL345.h"

class SensorManager
{
public:
    struct SensorData
    {
        ICM20948::SensorData imu;

        GY271::MagneticField magnetometer;

        BMP280::Measurements barometer;

        ADXL345::Acceleration externalAccelerometer;
    };

    SensorManager(
        ICM20948 &imu,
        GY271 &magnetometer,
        BMP280 &barometer,
        ADXL345 &accelerometer);

    bool initialize();

    bool read(
        SensorData &data);

    bool readIMU(
        ICM20948::SensorData &data);

    bool readMagnetometer(
        GY271::MagneticField &data);

    bool readBarometer(
        BMP280::Measurements &data);

    bool readAccelerometer(
        ADXL345::Acceleration &data);

    bool isIMUConnected();

    bool isMagnetometerConnected();

    bool isBarometerConnected();

    bool isAccelerometerConnected();

private:
    ICM20948 &imu;
    GY271 &magnetometer;
    BMP280 &barometer;
    ADXL345 &accelerometer;

    bool initialized = false;
};