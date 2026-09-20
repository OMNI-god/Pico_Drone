#pragma once

#include <cstdint>

enum class SafetyState : std::uint8_t
{
    Disarmed = 0,
    Arming,
    Armed,
    Failsafe
};

enum class SafetyFault : std::uint32_t
{
    None = 0,
    RcInvalid = 1u << 0,
    RcFailsafe = 1u << 1,
    RcUnhealthy = 1u << 2,
    ThrottleNotLow = 1u << 3,
    ImuInvalid = 1u << 4,
    ImuNotCalibrated = 1u << 5,
    ImuDataStale = 1u << 6,
    ArmSwitchNotActive = 1u << 7
};

inline SafetyFault operator|(
    SafetyFault lhs,
    SafetyFault rhs)
{
    return static_cast<SafetyFault>(
        static_cast<std::uint32_t>(lhs) |
        static_cast<std::uint32_t>(rhs));
}

inline SafetyFault operator&(
    SafetyFault lhs,
    SafetyFault rhs)
{
    return static_cast<SafetyFault>(
        static_cast<std::uint32_t>(lhs) &
        static_cast<std::uint32_t>(rhs));
}

inline bool hasFault(
    SafetyFault faults,
    SafetyFault fault)
{
    return (
               static_cast<std::uint32_t>(faults) &
               static_cast<std::uint32_t>(fault)) != 0;
}