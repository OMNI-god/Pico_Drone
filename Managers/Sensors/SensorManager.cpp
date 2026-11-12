#include "SensorManager.h"

#include <cstdio>

SensorManager::SensorManager(
    ICM20948 &imu,
    GY271 &magnetometer,
    BMP280 &barometer,
    ADXL345 &accelerometer,
    Gps &gps)
    : imu(imu),
      magnetometer(magnetometer),
      barometer(barometer),
      accelerometer(accelerometer),
      gps(gps)
{
}

// ======================================================
// Initialize
// ======================================================

bool SensorManager::initialize()
{
    initialized = false;

    imuHealthy = false;
    magnetometerHealthy = false;
    barometerHealthy = false;
    accelerometerHealthy = false;
    gpsHealthy = false;

    printf("\n");
    printf("Initializing sensors...\n");

    // ==================================================
    // ICM20948 - mandatory
    // ==================================================

    printf("ICM20948: ");

    if (!imu.isConnected())
    {
        printf("NOT CONNECTED\n");
    }
    else if (!imu.initialize())
    {
        printf("INITIALIZATION FAILED\n");
    }
    else
    {
        printf("OK\n");

        imuHealthy = true;
    }

    // ==================================================
    // GY271 - optional
    // ==================================================

    printf("GY271: ");

    if (!magnetometer.isConnected())
    {
        printf("NOT CONNECTED\n");
    }
    else if (!magnetometer.initialize())
    {
        printf("INITIALIZATION FAILED\n");
    }
    else
    {
        printf("OK\n");

        magnetometerHealthy = true;
    }

    // ==================================================
    // BMP280 - optional
    // ==================================================

    printf("BMP280: ");

    if (!barometer.isConnected())
    {
        printf("NOT CONNECTED\n");
    }
    else if (!barometer.initialize())
    {
        printf("INITIALIZATION FAILED\n");
    }
    else
    {
        printf("OK\n");

        barometerHealthy = true;
    }

    // ==================================================
    // ADXL345 - optional
    // ==================================================

    printf("ADXL345: ");

    if (!accelerometer.isConnected())
    {
        printf("NOT CONNECTED\n");
    }
    else if (!accelerometer.initialize())
    {
        printf("INITIALIZATION FAILED\n");
    }
    else
    {
        printf("OK\n");

        accelerometerHealthy = true;
    }

    // ==================================================
    // GPS - optional
    // ==================================================

    printf("GPS: ");

    if (!gps.initialize())
    {
        printf("INITIALIZATION FAILED\n");
    }
    else
    {
        printf("OK\n");

        gpsHealthy = true;
    }

    initialized = imuHealthy;

    if (initialized)
    {
        printf(
            "Sensor manager ready (IMU OK)\n");
    }
    else
    {
        printf(
            "ERROR: IMU is not available\n");
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
    // IMU - mandatory
    // ==================================================

    if (!imu.read(data.imu))
    {
        return false;
    }

    // ==================================================
    // Magnetometer - optional
    // ==================================================

    if (!magnetometer.read(
            data.magnetometer))
    {
        // The magnetometer may simply not have a new
        // sample yet. Keep the previous value.
    }

    // ==================================================
    // BMP280 - optional
    // ==================================================

    if (!barometer.read(
            data.barometer))
    {
        // The barometer updates at its own rate.
        // Keep the previous value.
    }

    // ==================================================
    // External ADXL345 - optional
    // ==================================================

    if (!accelerometer.readAcceleration(
            data.externalAccelerometer))
    {
        // Keep the previous value.
    }

    // ==================================================
    // GPS - optional
    // ==================================================

    readGps(data.gps);

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
    if (!initialized ||
        !magnetometerHealthy)
    {
        return false;
    }

    return magnetometer.read(data);
}

bool SensorManager::readBarometer(
    BMP280::Measurements &data)
{
    if (!initialized ||
        !barometerHealthy)
    {
        return false;
    }

    return barometer.read(data);
}

bool SensorManager::readAccelerometer(
    ADXL345::Acceleration &data)
{
    if (!initialized ||
        !accelerometerHealthy)
    {
        return false;
    }

    return accelerometer.readAcceleration(data);
}

bool SensorManager::readGps(
    Gps::GpsState &data)
{
    if (!initialized ||
        !gpsHealthy)
    {
        return false;
    }

    return gps.read(data);
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

bool SensorManager::isGpsConnected()
{
    return gps.isInitialized();
}

// ======================================================
// Health status
// ======================================================

bool SensorManager::isIMUHealthy() const
{
    return imuHealthy;
}

bool SensorManager::isMagnetometerHealthy() const
{
    return magnetometerHealthy;
}

bool SensorManager::isBarometerHealthy() const
{
    return barometerHealthy;
}

bool SensorManager::isAccelerometerHealthy() const
{
    return accelerometerHealthy;
}

bool SensorManager::isGpsHealthy() const
{
    return gpsHealthy;
}
