#include "IMUCalibration.h"

#include <cmath>

IMUCalibration::IMUCalibration()
    : IMUCalibration(Configuration{})
{
}

IMUCalibration::IMUCalibration(const Configuration &config)
    : config_(config),
      sampleCount_(0),
      gyroBiasX_(0.0f),
      gyroBiasY_(0.0f),
      gyroBiasZ_(0.0f),
      gyroSumX_(0.0f),
      gyroSumY_(0.0f),
      gyroSumZ_(0.0f),
      calibrated_(false)
{
}

void IMUCalibration::reset()
{
    sampleCount_ = 0;

    gyroBiasX_ = 0.0f;
    gyroBiasY_ = 0.0f;
    gyroBiasZ_ = 0.0f;

    gyroSumX_ = 0.0f;
    gyroSumY_ = 0.0f;
    gyroSumZ_ = 0.0f;

    calibrated_ = false;
}

bool IMUCalibration::isStationary(const IMUState &sample) const
{
    if (!sample.valid)
    {
        return false;
    }

    const float accelMagnitude =
        std::sqrt(
            sample.accelX * sample.accelX +
            sample.accelY * sample.accelY +
            sample.accelZ * sample.accelZ);

    const float accelError =
        std::fabs(accelMagnitude - 1.0f);

    const float gyroMagnitude =
        std::sqrt(
            sample.gyroX * sample.gyroX +
            sample.gyroY * sample.gyroY +
            sample.gyroZ * sample.gyroZ);

    return accelError <= config_.stationaryAccelTolerance &&
           gyroMagnitude <= config_.stationaryGyroTolerance;
}

bool IMUCalibration::addSample(const IMUState &sample)
{
    if (calibrated_)
    {
        return true;
    }

    if (!isStationary(sample))
    {
        return false;
    }

    gyroSumX_ += sample.gyroX;
    gyroSumY_ += sample.gyroY;
    gyroSumZ_ += sample.gyroZ;

    ++sampleCount_;

    if (sampleCount_ >= config_.sampleCount)
    {
        const float count =
            static_cast<float>(sampleCount_);

        gyroBiasX_ = gyroSumX_ / count;
        gyroBiasY_ = gyroSumY_ / count;
        gyroBiasZ_ = gyroSumZ_ / count;

        calibrated_ = true;
    }

    return true;
}

bool IMUCalibration::isComplete() const
{
    return sampleCount_ >= config_.sampleCount;
}

bool IMUCalibration::isCalibrated() const
{
    return calibrated_;
}

void IMUCalibration::apply(IMUState &sample) const
{
    if (!calibrated_)
    {
        return;
    }

    sample.gyroX -= gyroBiasX_;
    sample.gyroY -= gyroBiasY_;
    sample.gyroZ -= gyroBiasZ_;
}

const IMUState &IMUCalibration::getGyroBias() const
{
    static IMUState bias{};

    bias.gyroX = gyroBiasX_;
    bias.gyroY = gyroBiasY_;
    bias.gyroZ = gyroBiasZ_;

    bias.valid = calibrated_;

    return bias;
}

std::uint16_t IMUCalibration::getSampleCount() const
{
    return sampleCount_;
}

std::uint16_t IMUCalibration::getRequiredSampleCount() const
{
    return config_.sampleCount;
}