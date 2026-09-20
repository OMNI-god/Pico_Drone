#pragma once

#include <cstdint>

struct RCHealth
{
    // ------------------------------------------------------------
    // Link state
    // ------------------------------------------------------------

    bool connected = false;
    bool failsafe = true;
    bool healthy = false;

    // ------------------------------------------------------------
    // Frame statistics
    // ------------------------------------------------------------

    std::uint32_t totalFrames = 0;
    std::uint32_t framesLastSecond = 0;

    // ------------------------------------------------------------
    // Timing
    // ------------------------------------------------------------

    std::uint64_t lastFrameTimeUs = 0;
    std::uint64_t frameAgeUs = 0;

    // ------------------------------------------------------------
    // Input statistics
    // ------------------------------------------------------------

    std::uint32_t totalBytes = 0;

    // ------------------------------------------------------------
    // Health thresholds
    // ------------------------------------------------------------

    static constexpr std::uint32_t MinimumFrameRate = 20;
    static constexpr std::uint64_t MaximumFrameAgeUs = 100'000;
};