#pragma once

#include <cstdint>

// ============================================================
// IMU STATE
// ICM20948 accelerometer + gyroscope
// ============================================================

struct IMUState
{
    // Acceleration in g
    float accelX = 0.0f;
    float accelY = 0.0f;
    float accelZ = 0.0f;

    // Angular velocity in degrees per second
    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;

    // Timestamp of the sensor sample, in microseconds
    uint64_t timestampUs = 0;

    bool valid = false;
};

// ============================================================
// MAGNETOMETER STATE
// GY-271
// ============================================================

struct MagnetometerState
{
    // Magnetic field strength in Gauss
    float magX = 0.0f;
    float magY = 0.0f;
    float magZ = 0.0f;

    uint64_t timestampUs = 0;

    bool valid = false;
};

// ============================================================
// BAROMETER STATE
// BMP280
// ============================================================

struct BarometerState
{
    // Temperature in degrees Celsius
    float temperatureC = 0.0f;

    // Atmospheric pressure in Pascals
    float pressurePa = 0.0f;

    // Atmospheric pressure in hectopascals
    float pressureHpa = 0.0f;

    // Estimated altitude in meters
    float altitudeM = 0.0f;

    uint64_t timestampUs = 0;

    bool valid = false;
};

// ============================================================
// AGGREGATED SENSOR STATE
// ============================================================

struct SensorState
{
    IMUState imu{};
    MagnetometerState magnetometer{};
    BarometerState barometer{};

    // Sensor health / availability flags
    bool imuHealthy = false;
    bool magnetometerHealthy = false;
    bool barometerHealthy = false;
};