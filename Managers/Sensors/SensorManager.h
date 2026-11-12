#pragma once

#include "ICM20948.h"
#include "GY271.h"
#include "BMP280.h"
#include "ADXL345.h"
#include "Gps.h"

class SensorManager
{
public:
    struct SensorData
    {
        ICM20948::SensorData imu;

        GY271::MagneticField magnetometer;

        BMP280::Measurements barometer;

        ADXL345::Acceleration externalAccelerometer;

        Gps::GpsState gps;
    };

    SensorManager(
        ICM20948 &imu,
        GY271 &magnetometer,
        BMP280 &barometer,
        ADXL345 &accelerometer,
        Gps &gps);

    /*
     * Initialize every sensor.
     *
     * The IMU is the only mandatory sensor. Every other
     * sensor is optional: a missing magnetometer,
     * barometer, accelerometer or GPS must not prevent
     * the flight controller from starting.
     *
     * Returns true when the IMU is operational.
     */
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

    bool readGps(
        Gps::GpsState &data);

    bool isIMUConnected();

    bool isMagnetometerConnected();

    bool isBarometerConnected();

    bool isAccelerometerConnected();

    bool isGpsConnected();

    bool isIMUHealthy() const;

    bool isMagnetometerHealthy() const;

    bool isBarometerHealthy() const;

    bool isAccelerometerHealthy() const;

    bool isGpsHealthy() const;

private:
    ICM20948 &imu;
    GY271 &magnetometer;
    BMP280 &barometer;
    ADXL345 &accelerometer;
    Gps &gps;

    bool initialized = false;

    bool imuHealthy = false;

    bool magnetometerHealthy = false;

    bool barometerHealthy = false;

    bool accelerometerHealthy = false;

    bool gpsHealthy = false;
};
