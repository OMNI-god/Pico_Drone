#pragma once

#include <cstdint>

#include "IMUSnapshot.h"

class IMUPublisher
{
public:
    IMUPublisher();

    void publish(const IMUSnapshot &snapshot);

    bool read(IMUSnapshot &snapshot) const;

private:
    IMUSnapshot snapshot_;

    volatile std::uint32_t sequence_;
};