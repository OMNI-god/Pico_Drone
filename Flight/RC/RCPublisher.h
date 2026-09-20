#pragma once

#include <cstdint>

#include "RCState.h"

class RCPublisher
{
public:
    RCPublisher();

    void publish(
        const RCState &state);

    bool read(
        RCState &state) const;

private:
    RCState _state;

    volatile std::uint32_t _sequence;
};