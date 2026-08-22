#include "SensorManager.h"

#include <cstdio>

SensorManager::SensorManager(
    ICM20948 &imu,
    GY271 &magnetometer,
    BMP280 &barometer,
    ADXL345 &accelerometer)
    : imu(imu),
      magnetometer(magnetometer),
      barometer(barometer),
      accelerometer(accelerometer)
{
}

// ======================================================
// Initialize
// ======================================================

bool SensorManager::initialize()
{
    initialized = false;

    bool success = true;

    printf("Initializing sensors...\n");

    // ==================================================
    // ICM20948
    // ==================================================

    printf("ICM20948: ");

    if (!imu.isConnected())
    {
        printf("NOT CONNECTED\n");
        success = false;
    }
    else if (!imu.initialize())
    {
        printf("INITIALIZATION FAILED\n");
        success = false;
    }
    else
    {
        printf("OK\n");
    }

    // ==================================================
    // GY271
    // ==================================================

    printf("GY271: ");

    if (!magnetometer.isConnected())
    {
        printf("NOT CONNECTED\n");
        success = false;
    }
    else if (!magnetometer.initialize())
    {
        printf("INITIALIZATION FAILED\n");
        success = false;
    }
    else
    {
        printf("OK\n");
    }

    // ==================================================
    // BMP280
    // ==================================================

    printf("BMP280: ");

    if (!barometer.isConnected())
    {
        printf("NOT CONNECTED\n");
        success = false;
    }
    else if (!barometer.initialize())
    {
        printf("INITIALIZATION FAILED\n");
        success = false;
    }
    else
    {
        printf("OK\n");
    }

    // ==================================================
    // ADXL345
    // ==================================================

    printf("ADXL345: ");

    if (!accelerometer.isConnected())
    {
        printf("NOT CONNECTED\n");
        success = false;
    }
    else if (!accelerometer.initialize())
    {
        printf("INITIALIZATION FAILED\n");
        success = false;
    }
    else
    {
        printf("OK\n");
    }

    initialized = success;

    if (initialized)
    {
        printf(
            "All sensors initialized successfully\n");
    }
    else
    {
        printf(
            "One or more sensors failed\n");
    }

    return initialized;
}

// ======================================================
// Read all sensors
// ======================================================

bool SensorManager::read(
    SensorData &data)
{
    if (!initialized)
    {
        return false;
    }

    // ==================================================
    // IMU
    // ==================================================

    if (!imu.read(data.imu))
    {
        printf("IMU READ FAILED\n");
        return false;
    }

    // ==================================================
    // Magnetometer
    // ==================================================

    if (!magnetometer.read(
            data.magnetometer))
    {
        // Magnetometer may simply not have a new sample yet.
        //
        // Do NOT fail the entire sensor manager.
        //
        // Keep the previous magnetometer values.

        printf("MAG NOT READY\n");
    }
    else
    {
        printf(
            "MAG: X=%.4f Y=%.4f Z=%.4f\n",
            data.magnetometer.x,
            data.magnetometer.y,
            data.magnetometer.z);
    }

    // ==================================================
    // BMP280
    // ==================================================

    if (!barometer.read(
            data.barometer))
    {
        printf("BARO READ FAILED\n");
        return false;
    }

    // ==================================================
    // External ADXL345
    // ==================================================

    if (!accelerometer.readAcceleration(
            data.externalAccelerometer))
    {
        printf("ADXL READ FAILED\n");
        return false;
    }

    return true;
}

// ======================================================
// Individual sensor reads
// ======================================================

bool SensorManager::readIMU(
    ICM20948::SensorData &data)
{
    if (!initialized)
    {
        return false;
    }

    return imu.read(data);
}

bool SensorManager::readMagnetometer(
    GY271::MagneticField &data)
{
    if (!initialized)
    {
        return false;
    }

    return magnetometer.read(data);
}

bool SensorManager::readBarometer(
    BMP280::Measurements &data)
{
    if (!initialized)
    {
        return false;
    }

    return barometer.read(data);
}

bool SensorManager::readAccelerometer(
    ADXL345::Acceleration &data)
{
    if (!initialized)
    {
        return false;
    }

    return accelerometer.readAcceleration(data);
}

// ======================================================
// Connection status
// ======================================================

bool SensorManager::isIMUConnected()
{
    return imu.isConnected();
}

bool SensorManager::isMagnetometerConnected()
{
    return magnetometer.isConnected();
}

bool SensorManager::isBarometerConnected()
{
    return barometer.isConnected();
}

bool SensorManager::isAccelerometerConnected()
{
    return accelerometer.isConnected();
}