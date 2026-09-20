#pragma once

#include "SafetySnapshot.h"

class SafetyPublisher
{
public:
    SafetyPublisher();

    void publish(const SafetySnapshot &snapshot);

    bool read(SafetySnapshot &snapshot) const;

private:
    SafetySnapshot _snapshot{};
    volatile std::uint32_t _sequence = 0;
};