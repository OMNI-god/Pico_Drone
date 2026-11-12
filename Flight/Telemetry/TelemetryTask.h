#pragma once

#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"

#include "Elrs.h"
#include "CrsfTelemetry.h"
#include "RCPublisher.h"
#include "IMUPublisher.h"
#include "SensorManager.h"
#include "FlightController.h"

/*
 * Telemetry task.
 *
 * Collects the flight state and converts it into
 * CRSF telemetry frames:
 *
 *     RCPublisher      ->  RC state
 *     IMUPublisher     ->  IMU snapshot
 *     SensorManager    ->  GPS / barometer
 *     FlightController ->  attitude / arming state
 *     CrsfTelemetry    ->  CRSF frames
 *     Elrs             ->  transmitter
 */
class TelemetryTask
{
public:
    struct Configuration
    {
        std::uint32_t updateRateHz;

        std::uint32_t stackDepth;

        std::uint32_t priority;

        /*
         * Vertical speed low pass filter gain.
         */
        float verticalSpeedFilter;

        Configuration()
            : updateRateHz(100),
              stackDepth(2048),
              priority(4),
              verticalSpeedFilter(0.2f)
        {
        }
    };

    TelemetryTask(
        RCPublisher &rcPublisher,
        IMUPublisher &imuPublisher,
        SensorManager &sensors,
        FlightController &flightController,
        CrsfTelemetry &telemetry,
        Elrs &elrs,
        const Configuration &configuration = Configuration());

    bool start(
        const char *taskName = "Telemetry",
        std::uint32_t stackDepth = 0,
        std::uint32_t priority = 0);

    /*
     * Thread-safe telemetry state copy.
     */
    bool readState(
        TelemetryState &state) const;

private:
    static void taskEntry(
        void *parameter);

    void run();

    void updateState(
        const RCState &rc,
        const IMUSnapshot &imu,
        const FlightControllerState &flightControllerState,
        std::uint32_t nowMs);

    void updateVerticalSpeed(
        float altitudeM,
        std::uint32_t nowMs);

    void transmit();

private:
    RCPublisher &_rcPublisher;
    IMUPublisher &_imuPublisher;
    SensorManager &_sensors;

    FlightController &_flightController;

    CrsfTelemetry &_telemetry;
    Elrs &_elrs;

    Configuration _configuration;

    /*
     * Latest sensor snapshots.
     */
    Gps::GpsState _gpsState{};

    BMP280::Measurements _barometer{};

    bool _barometerValid = false;

    /*
     * Published telemetry state.
     */
    TelemetryState _state;

    /*
     * Vertical speed estimation.
     */
    float _lastAltitudeM = 0.0f;
    std::uint32_t _lastAltitudeTimeMs = 0;

    TaskHandle_t _taskHandle = nullptr;
};
