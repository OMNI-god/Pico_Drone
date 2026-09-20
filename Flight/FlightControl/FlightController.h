#pragma once

#include <cstdint>

#include "RCState.h"
#include "SensorState.h"
#include "IMUSnapshot.h"

struct FlightControllerState
{
    // ------------------------------------------------------------
    // RC commands
    // ------------------------------------------------------------

    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float throttle = 0.0f;

    // ------------------------------------------------------------
    // Flight state
    // ------------------------------------------------------------

    bool armed = false;
    bool failsafe = true;

    RCFlightMode flightMode =
        RCFlightMode::Angle;

    // ------------------------------------------------------------
    // Sensor state
    // ------------------------------------------------------------

    bool imuHealthy = false;

    // ------------------------------------------------------------
    // Timing
    // ------------------------------------------------------------

    std::uint64_t lastUpdateUs = 0;
};

class FlightController
{
public:
    FlightController();

    void update(
        const RCState &rc,
        std::uint64_t nowUs);

    void update(
        const RCState &rc,
        const IMUSnapshot &imu,
        std::uint64_t nowUs);

    const FlightControllerState &getState() const;

private:
    void applyFailsafe();

private:
    FlightControllerState _state;
};