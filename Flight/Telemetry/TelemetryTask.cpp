#include "TelemetryTask.h"

#include "pico/stdlib.h"
#include "pico/time.h"

#include <cmath>

// ============================================================
// Constructor
// ============================================================

TelemetryTask::TelemetryTask(
    RCPublisher &rcPublisher,
    IMUPublisher &imuPublisher,
    SensorManager &sensors,
    FlightController &flightController,
    CrsfTelemetry &telemetry,
    Elrs &elrs,
    const Configuration &configuration)
    : _rcPublisher(rcPublisher),
      _imuPublisher(imuPublisher),
      _sensors(sensors),
      _flightController(flightController),
      _telemetry(telemetry),
      _elrs(elrs),
      _configuration(configuration)
{
}

// ============================================================
// Start
// ============================================================

bool TelemetryTask::start(
    const char *taskName,
    std::uint32_t stackDepth,
    std::uint32_t priority)
{
    if (_taskHandle != nullptr)
    {
        return false;
    }

    const std::uint32_t actualStackDepth =
        stackDepth == 0 ?
            _configuration.stackDepth :
            stackDepth;

    const std::uint32_t actualPriority =
        priority == 0 ?
            _configuration.priority :
            priority;

    const BaseType_t result =
        xTaskCreate(
            taskEntry,
            taskName,
            actualStackDepth,
            this,
            static_cast<UBaseType_t>(actualPriority),
            &_taskHandle);

    return result == pdPASS;
}

// ============================================================
// Task entry
// ============================================================

void TelemetryTask::taskEntry(
    void *parameter)
{
    auto *task =
        static_cast<TelemetryTask *>(parameter);

    if (task == nullptr)
    {
        vTaskDelete(nullptr);
        return;
    }

    task->run();

    vTaskDelete(nullptr);
}

// ============================================================
// Main loop
// ============================================================

void TelemetryTask::run()
{
    /*
     * GPS service interval.
     *
     * The GPS receiver streams NMEA at 115200 baud.
     * The PIO receiver stores bytes in a ring buffer,
     * which the telemetry task drains here. Polling
     * every millisecond keeps the buffer nearly empty
     * and lets the NMEA parser keep up with the stream
     * while the flight/CRSF state still publishes at
     * the configured rate.
     */

    constexpr TickType_t GpsServicePeriodTicks = 1;

    TickType_t telemetryPeriodTicks =
        pdMS_TO_TICKS(
            1000 / _configuration.updateRateHz);

    if (telemetryPeriodTicks == 0)
    {
        telemetryPeriodTicks = 1;
    }

    TickType_t lastWakeTime =
        xTaskGetTickCount();

    TickType_t lastTelemetryTicks =
        lastWakeTime;

    /*
     * The barometer updates at roughly 1 Hz, so there is
     * no need to poll it at the full telemetry rate.
     */

    constexpr std::uint32_t BarometerIntervalMs = 500;

    std::uint32_t lastBarometerMs = 0;

    while (true)
    {
        /*
         * GPS fast path.
         *
         * The GPS driver is owned by this task: it is
         * the only consumer polling the receiver.
         */

        Gps::GpsState gpsState{};

        (void)_sensors.readGps(gpsState);

        _gpsState = gpsState;

        const TickType_t nowTicks =
            xTaskGetTickCount();

        if ((nowTicks - lastTelemetryTicks) >=
            telemetryPeriodTicks)
        {
            lastTelemetryTicks = nowTicks;

            RCState rc{};
            IMUSnapshot imu{};
            FlightControllerState flightControllerState{};

            (void)_rcPublisher.read(rc);
            (void)_imuPublisher.read(imu);
            (void)_flightController.readState(
                flightControllerState);

            const std::uint32_t nowMs =
                static_cast<std::uint32_t>(
                    time_us_64() / 1000ULL);

            /*
             * Barometer.
             */

            if (nowMs - lastBarometerMs >=
                BarometerIntervalMs)
            {
                BMP280::Measurements barometer{};

                const bool barometerValid =
                    _sensors.readBarometer(barometer);

                if (barometerValid)
                {
                    _barometer = barometer;
                    _barometerValid = true;

                    updateVerticalSpeed(
                        barometer.altitudeM,
                        nowMs);
                }

                lastBarometerMs = nowMs;
            }

            /*
             * Flight state.
             */

            updateState(
                rc,
                imu,
                flightControllerState,
                nowMs);

            /*
             * CRSF telemetry.
             */

            _telemetry.update(
                _state,
                nowMs);

            transmit();
        }

        vTaskDelayUntil(
            &lastWakeTime,
            GpsServicePeriodTicks);
    }
}

// ============================================================
// State assembly
// ============================================================

void TelemetryTask::updateState(
    const RCState &rc,
    const IMUSnapshot &imu,
    const FlightControllerState &flightControllerState,
    std::uint32_t nowMs)
{
    TelemetryState newState = _state;

    (void)imu;

    /*
     * ---------------------------------------------------------
     * GPS
     * ---------------------------------------------------------
     */

    newState.latitude = _gpsState.latitude;
    newState.longitude = _gpsState.longitude;

    newState.gpsAltitude =
        _gpsState.altitudeM;

    newState.groundSpeed =
        _gpsState.groundSpeedMs;

    newState.gpsHeading =
        _gpsState.headingDeg;

    newState.satellites =
        _gpsState.satellites;

    /*
     * ---------------------------------------------------------
     * Attitude
     * ---------------------------------------------------------
     */

    newState.roll =
        flightControllerState.attitudeRoll;

    newState.pitch =
        flightControllerState.attitudePitch;

    newState.yaw =
        flightControllerState.attitudeYaw;

    /*
     * ---------------------------------------------------------
     * Altitude
     * ---------------------------------------------------------
     */

    if (_barometerValid)
    {
        newState.altitude =
            _barometer.altitudeM;
    }

    /*
     * ---------------------------------------------------------
     * Arming / flight mode
     * ---------------------------------------------------------
     */

    newState.armed =
        flightControllerState.armed;

    switch (rc.flightMode)
    {
    case RCFlightMode::Angle:
        newState.flightMode = 1;
        break;

    case RCFlightMode::Horizon:
        newState.flightMode = 2;
        break;

    case RCFlightMode::Acro:
        newState.flightMode = 0;
        break;

    default:
        newState.flightMode = 1;
        break;
    }

    /*
     * ---------------------------------------------------------
     * Timestamp
     * ---------------------------------------------------------
     */

    newState.updateTimeMs = nowMs;

    /*
     * ---------------------------------------------------------
     * Publish
     * ---------------------------------------------------------
     */

    taskENTER_CRITICAL();

    _state = newState;

    taskEXIT_CRITICAL();
}

// ============================================================
// Vertical speed
// ============================================================

void TelemetryTask::updateVerticalSpeed(
    float altitudeM,
    std::uint32_t nowMs)
{
    if (_lastAltitudeTimeMs == 0)
    {
        _lastAltitudeM = altitudeM;
        _lastAltitudeTimeMs = nowMs;

        return;
    }

    const std::uint32_t deltaMs =
        nowMs - _lastAltitudeTimeMs;

    if (deltaMs == 0)
    {
        return;
    }

    const float instantaneous =
        (altitudeM - _lastAltitudeM) *
        1000.0f /
        static_cast<float>(deltaMs);

    const float alpha =
        _configuration.verticalSpeedFilter;

    _state.verticalSpeed =
        (1.0f - alpha) * _state.verticalSpeed +
        alpha * instantaneous;

    _lastAltitudeM = altitudeM;
    _lastAltitudeTimeMs = nowMs;
}

// ============================================================
// Transmission
// ============================================================

void TelemetryTask::transmit()
{
    if (!_telemetry.hasFrame())
    {
        return;
    }

    const std::uint8_t *frame =
        _telemetry.frame();

    const std::uint8_t frameSize =
        _telemetry.frameSize();

    if (frame != nullptr &&
        frameSize > 0)
    {
        _elrs.write(
            frame,
            frameSize);
    }

    _telemetry.clearFrame();
}

// ============================================================
// State access
// ============================================================

bool TelemetryTask::readState(
    TelemetryState &state) const
{
    taskENTER_CRITICAL();

    state = _state;

    taskEXIT_CRITICAL();

    return true;
}
