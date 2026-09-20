#pragma once

#include <cstdint>

#include "SensorState.h"

class IMUCalibration
{
public:
    struct Configuration
    {
        std::uint16_t sampleCount;
        float stationaryAccelTolerance;
        float stationaryGyroTolerance;

        Configuration()
            : sampleCount(2000),
              stationaryAccelTolerance(0.15f),
              stationaryGyroTolerance(5.0f)
        {
        }
    };

public:
    IMUCalibration();

    explicit IMUCalibration(const Configuration &config);

    void reset();

    bool addSample(const IMUState &sample);

    bool isComplete() const;

    bool isCalibrated() const;

    void apply(IMUState &sample) const;

    const IMUState &getGyroBias() const;

    std::uint16_t getSampleCount() const;

    std::uint16_t getRequiredSampleCount() const;

private:
    bool isStationary(const IMUState &sample) const;

private:
    Configuration config_;

    std::uint16_t sampleCount_;

    float gyroBiasX_;
    float gyroBiasY_;
    float gyroBiasZ_;

    float gyroSumX_;
    float gyroSumY_;
    float gyroSumZ_;

    bool calibrated_;
};