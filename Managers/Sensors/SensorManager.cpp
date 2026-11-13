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

    /*
     * Probe first so a missing device can be reported
     * separately from an initialization failure.
     */

    const bool imuConnected =
        imu.isConnected();

    const bool magnetometerConnected =
        magnetometer.isConnected();

    const bool barometerConnected =
        barometer.isConnected();

    const bool accelerometerConnected =
        accelerometer.isConnected();

    /*
     * Each driver prints its own detection log. The
     * status table is printed afterwards so the logs
     * and the summary do not interleave.
     */

    imuHealthy =
        imuConnected && imu.initialize();

    magnetometerHealthy =
        magnetometerConnected && magnetometer.initialize();

    barometerHealthy =
        barometerConnected && barometer.initialize();

    accelerometerHealthy =
        accelerometerConnected && accelerometer.initialize();

    gpsHealthy =
        gps.initialize();

    const auto statusText = [](bool connected, bool healthy)
    {
        if (!connected)
        {
            return "NOT CONNECTED";
        }

        return healthy ? "OK" : "FAILED";
    };

    printf("\n");
    printf("      ICM20948       %s\n",
           statusText(imuConnected, imuHealthy));
    printf("      GY271          %s\n",
           statusText(magnetometerConnected, magnetometerHealthy));
    printf("      BMP280         %s\n",
           statusText(barometerConnected, barometerHealthy));
    printf("      ADXL345        %s\n",
           statusText(accelerometerConnected, accelerometerHealthy));
    printf("      GPS            %s\n",
           gpsHealthy ? "OK" : "FAILED");

    initialized = imuHealthy;

    if (initialized)
    {
        printf("      Sensor manager ready\n");
    }
    else
    {
        printf("      ERROR: IMU is not available\n");
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
