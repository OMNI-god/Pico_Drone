#pragma once

#include <cstdint>

/**
 * Complete telemetry state produced by the flight controller.
 *
 * This structure is independent of CRSF.
 *
 * Sensor/flight-control code writes telemetry data here,
 * while CrsfTelemetry converts this state into CRSF
 * telemetry frames.
 */
struct TelemetryState
{
    // ========================================================
    // Battery
    // ========================================================

    /**
     * Battery voltage in volts.
     *
     * Example:
     *     11.10 V
     */
    float batteryVoltage = 0.0f;

    /**
     * Battery current in amperes.
     *
     * Positive value represents current being consumed
     * by the flight controller / propulsion system.
     */
    float batteryCurrent = 0.0f;

    /**
     * Consumed battery capacity in mAh.
     */
    float batteryConsumedMah = 0.0f;

    /**
     * Remaining battery percentage.
     *
     * Valid range:
     *
     *     0 ... 100
     */
    uint8_t batteryRemaining = 0;

    // ========================================================
    // GPS
    // ========================================================

    /**
     * Latitude in decimal degrees.
     *
     * Range:
     *
     *     -90 ... +90
     */
    double latitude = 0.0;

    /**
     * Longitude in decimal degrees.
     *
     * Range:
     *
     *     -180 ... +180
     */
    double longitude = 0.0;

    /**
     * GPS altitude in meters.
     */
    float gpsAltitude = 0.0f;

    /**
     * Ground speed in km/h.
     */
    float groundSpeed = 0.0f;

    /**
     * GPS course/heading in degrees.
     *
     * Range:
     *
     *     0 ... <360
     */
    float gpsHeading = 0.0f;

    /**
     * Number of satellites currently tracked.
     */
    uint8_t satellites = 0;

    // ========================================================
    // Flight
    // ========================================================

    /**
     * Flight-controller altitude in meters.
     *
     * This may come from the barometer, GPS, or
     * the flight estimator depending on the architecture.
     */
    float altitude = 0.0f;

    /**
     * Vertical velocity in meters per second.
     *
     * Positive = climbing.
     * Negative = descending.
     */
    float verticalSpeed = 0.0f;

    /**
     * Roll angle in degrees.
     */
    float roll = 0.0f;

    /**
     * Pitch angle in degrees.
     */
    float pitch = 0.0f;

    /**
     * Yaw angle in degrees.
     */
    float yaw = 0.0f;

    // ========================================================
    // Home
    // ========================================================

    /**
     * Distance from the home position in meters.
     */
    float homeDistance = 0.0f;

    /**
     * Direction from the aircraft toward home in degrees.
     *
     * Range:
     *
     *     0 ... <360
     */
    float homeDirection = 0.0f;

    // ========================================================
    // System
    // ========================================================

    /**
     * Current armed state of the flight controller.
     */
    bool armed = false;

    /**
     * Flight mode identifier.
     *
     * Current mapping:
     *
     *     0 = ACRO
     *     1 = ANGLE
     *     2 = HORIZON
     *     3 = GPS
     */
    uint8_t flightMode = 0;

    // ========================================================
    // Link
    // ========================================================

    /**
     * Link RSSI.
     *
     * Stored as a signed value because some CRSF
     * implementations represent RSSI as a negative
     * dBm value.
     */
    int16_t rssi = 0;

    /**
     * Link quality percentage.
     *
     * Range:
     *
     *     0 ... 100
     */
    uint8_t linkQuality = 0;

    /**
     * Signal-to-noise ratio in dB.
     *
     * May be negative.
     */
    int8_t snr = 0;

    // ========================================================
    // Timestamp
    // ========================================================

    /**
     * Timestamp of the most recent telemetry update.
     *
     * Unit:
     *     milliseconds
     */
    uint32_t updateTimeMs = 0;
};