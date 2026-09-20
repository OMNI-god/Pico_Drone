#pragma once

#include <cstdint>

#include "RCState.h"
#include "RCHealth.h"
#include "IMUSnapshot.h"
#include "SafetySnapshot.h"

class SafetyManager
{
public:
    SafetyManager();

    void update(
        const RCState &rc,
        const RCHealth &rcHealth,
        const IMUSnapshot &imu,
        std::uint64_t nowUs);

    const SafetySnapshot &getSnapshot() const;

    bool isArmed() const;

    bool canArm() const;

private:
    void evaluateFaults(
        const RCState &rc,
        const RCHealth &rcHealth,
        const IMUSnapshot &imu,
        std::uint64_t nowUs);

    void updateState(
        const RCState &rc,
        std::uint64_t nowUs);

    bool areSafetyConditionsValid() const;

    bool isFailsafeRecoveryValid(
        const RCState &rc,
        std::uint64_t nowUs);

private:
    SafetySnapshot _snapshot{};

    SafetyState _previousState = SafetyState::Disarmed;

    std::uint64_t _lastUpdateUs = 0;

    /*
     * Timestamp when all failsafe recovery conditions
     * first became valid.
     *
     * 0 means recovery conditions are currently not valid.
     */
    std::uint64_t _failsafeRecoveryStartUs = 0;

    /*
     * IMU data must not be older than this.
     */
    static constexpr std::uint64_t ImuMaximumAgeUs = 20'000;

    /*
     * Throttle must be below 5% to permit arming
     * or failsafe recovery.
     */
    static constexpr float ThrottleArmLimit = 0.05f;

    /*
     * Safety conditions must remain continuously valid
     * for 500 ms before FAILSAFE can be cleared.
     */
    static constexpr std::uint64_t FailsafeRecoveryTimeUs = 500'000;
};