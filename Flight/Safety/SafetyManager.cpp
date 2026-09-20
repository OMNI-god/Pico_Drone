#include "SafetyManager.h"

SafetyManager::SafetyManager()
{
    _snapshot.state = SafetyState::Disarmed;
    _snapshot.faults = SafetyFault::None;

    _failsafeRecoveryStartUs = 0;
}

void SafetyManager::update(
    const RCState &rc,
    const RCHealth &rcHealth,
    const IMUSnapshot &imu,
    std::uint64_t nowUs)
{
    _lastUpdateUs = nowUs;

    _snapshot.timestampUs = nowUs;

    /*
     * ---------------------------------------------------------
     * RC
     * ---------------------------------------------------------
     */

    _snapshot.rcValid = rc.valid;
    _snapshot.rcFailsafe = rc.failsafe;
    _snapshot.rcHealthy = rcHealth.healthy;

    /*
     * ---------------------------------------------------------
     * Throttle / arming switch
     * ---------------------------------------------------------
     */

    _snapshot.throttleLow =
        rc.throttle <= ThrottleArmLimit;

    _snapshot.armRequested =
        rc.arm;

    /*
     * ---------------------------------------------------------
     * IMU
     * ---------------------------------------------------------
     */

    _snapshot.imuValid =
        imu.state.valid;

    _snapshot.imuCalibrated =
        imu.calibrated;

    _snapshot.imuDataFresh =
        imu.state.valid &&
        nowUs >= imu.state.timestampUs &&
        (nowUs - imu.state.timestampUs) <= ImuMaximumAgeUs;

    /*
     * ---------------------------------------------------------
     * Evaluate faults
     * ---------------------------------------------------------
     */

    evaluateFaults(
        rc,
        rcHealth,
        imu,
        nowUs);

    /*
     * ---------------------------------------------------------
     * Update state machine
     * ---------------------------------------------------------
     */

    updateState(
        rc,
        nowUs);
}

void SafetyManager::evaluateFaults(
    const RCState &rc,
    const RCHealth &rcHealth,
    const IMUSnapshot &imu,
    std::uint64_t nowUs)
{
    (void)nowUs;

    SafetyFault faults = SafetyFault::None;

    /*
     * RC validity
     */
    if (!rc.valid)
    {
        faults =
            faults | SafetyFault::RcInvalid;
    }

    /*
     * RC failsafe
     */
    if (rc.failsafe)
    {
        faults =
            faults | SafetyFault::RcFailsafe;
    }

    /*
     * RC health
     */
    if (!rcHealth.healthy)
    {
        faults =
            faults | SafetyFault::RcUnhealthy;
    }

    /*
     * Throttle
     */
    if (rc.throttle > ThrottleArmLimit)
    {
        faults =
            faults | SafetyFault::ThrottleNotLow;
    }

    /*
     * IMU validity
     */
    if (!imu.state.valid)
    {
        faults =
            faults | SafetyFault::ImuInvalid;
    }

    /*
     * IMU calibration
     */
    if (!imu.calibrated)
    {
        faults =
            faults | SafetyFault::ImuNotCalibrated;
    }

    /*
     * IMU freshness
     */
    if (!_snapshot.imuDataFresh)
    {
        faults =
            faults | SafetyFault::ImuDataStale;
    }

    /*
     * Arm switch
     *
     * This is intentionally represented as a fault because
     * the snapshot should expose why the system cannot arm.
     */
    if (!rc.arm)
    {
        faults =
            faults | SafetyFault::ArmSwitchNotActive;
    }

    _snapshot.faults = faults;
}

bool SafetyManager::areSafetyConditionsValid() const
{
    return _snapshot.rcValid &&
           !_snapshot.rcFailsafe &&
           _snapshot.rcHealthy &&
           _snapshot.throttleLow &&
           _snapshot.imuValid &&
           _snapshot.imuCalibrated &&
           _snapshot.imuDataFresh;
}

bool SafetyManager::isFailsafeRecoveryValid(
    const RCState &rc,
    std::uint64_t nowUs)
{
    /*
     * Recovery requires the normal safety conditions AND
     * the arm switch to be OFF.
     *
     * This is critical.
     *
     * We never want:
     *
     * FAILSAFE -> ARMED
     *
     * after RC recovery.
     */
    const bool recoveryConditions =
        areSafetyConditionsValid() &&
        !rc.arm;

    if (!recoveryConditions)
    {
        /*
         * Conditions became invalid again.
         *
         * Restart the recovery timer from zero.
         */
        _failsafeRecoveryStartUs = 0;

        return false;
    }

    /*
     * Start the recovery timer.
     */
    if (_failsafeRecoveryStartUs == 0)
    {
        _failsafeRecoveryStartUs = nowUs;

        return false;
    }

    /*
     * Protect against timestamp wrap / invalid ordering.
     */
    if (nowUs < _failsafeRecoveryStartUs)
    {
        _failsafeRecoveryStartUs = nowUs;

        return false;
    }

    /*
     * Require continuous healthy conditions for the
     * configured recovery period.
     */
    return (nowUs - _failsafeRecoveryStartUs) >=
           FailsafeRecoveryTimeUs;
}

void SafetyManager::updateState(
    const RCState &rc,
    std::uint64_t nowUs)
{
    const bool safetyConditionsValid =
        areSafetyConditionsValid();

    const bool armRequested =
        rc.arm;

    SafetyState newState =
        _snapshot.state;

    switch (_snapshot.state)
    {
    /*
     * -----------------------------------------------------
     * DISARMED
     * -----------------------------------------------------
     */
    case SafetyState::Disarmed:

        /*
         * Cannot arm until every safety condition
         * is valid.
         */
        if (!safetyConditionsValid)
        {
            newState =
                SafetyState::Disarmed;
        }
        else if (armRequested)
        {
            newState =
                SafetyState::Arming;
        }

        break;

    /*
     * -----------------------------------------------------
     * ARMING
     * -----------------------------------------------------
     */
    case SafetyState::Arming:

        /*
         * Any safety failure during arming immediately
         * aborts the arming process.
         */
        if (!safetyConditionsValid)
        {
            newState =
                SafetyState::Failsafe;
        }
        else if (!armRequested)
        {
            newState =
                SafetyState::Disarmed;
        }
        else
        {
            /*
             * Current implementation performs the
             * arming transition immediately once the
             * conditions are valid.
             *
             * A future version can add an arming
             * delay / stick gesture validation here.
             */
            newState =
                SafetyState::Armed;
        }

        break;

    /*
     * -----------------------------------------------------
     * ARMED
     * -----------------------------------------------------
     */
    case SafetyState::Armed:

        /*
         * Any critical safety failure causes immediate
         * transition to FAILSAFE.
         */
        if (!safetyConditionsValid)
        {
            newState =
                SafetyState::Failsafe;
        }
        else if (!armRequested)
        {
            /*
             * Normal pilot disarm.
             */
            newState =
                SafetyState::Disarmed;
        }

        break;

    /*
     * -----------------------------------------------------
     * FAILSAFE
     * -----------------------------------------------------
     */
    case SafetyState::Failsafe:

        /*
         * FAILSAFE IS LATCHED.
         *
         * We never automatically transition out of
         * FAILSAFE on the next update.
         *
         * Recovery requires:
         *
         * 1. RC valid
         * 2. RC not in failsafe
         * 3. RC healthy
         * 4. throttle low
         * 5. IMU valid
         * 6. IMU calibrated
         * 7. IMU data fresh
         * 8. ARM switch OFF
         * 9. all conditions continuously valid
         *    for FailsafeRecoveryTimeUs
         */
        if (isFailsafeRecoveryValid(rc, nowUs))
        {
            newState =
                SafetyState::Disarmed;
        }
        else
        {
            newState =
                SafetyState::Failsafe;
        }

        break;
    }

    /*
     * ---------------------------------------------------------
     * Transition accounting
     * ---------------------------------------------------------
     */

    if (newState != _snapshot.state)
    {
        ++_snapshot.transitionCounter;
    }

    _previousState =
        _snapshot.state;

    _snapshot.state =
        newState;

    /*
     * Once we leave FAILSAFE, make sure the recovery timer
     * is cleared.
     */
    if (_snapshot.state != SafetyState::Failsafe)
    {
        _failsafeRecoveryStartUs = 0;
    }
}

const SafetySnapshot &SafetyManager::getSnapshot() const
{
    return _snapshot;
}

bool SafetyManager::isArmed() const
{
    return _snapshot.state ==
           SafetyState::Armed;
}

bool SafetyManager::canArm() const
{
    return _snapshot.rcValid &&
           !_snapshot.rcFailsafe &&
           _snapshot.rcHealthy &&
           _snapshot.throttleLow &&
           _snapshot.imuValid &&
           _snapshot.imuCalibrated &&
           _snapshot.imuDataFresh &&
           _snapshot.armRequested;
}