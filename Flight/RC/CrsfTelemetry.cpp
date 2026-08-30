#include "CrsfTelemetry.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "CrsfCrc.h"

CrsfTelemetry::CrsfTelemetry()
    : _frameSize(0),
      _hasFrame(false),
      _lastBatteryMs(0),
      _lastGpsMs(0),
      _lastVarioMs(0),
      _lastAttitudeMs(0),
      _lastFlightModeMs(0)
{
}

// ============================================================================
// Update
// ============================================================================

void CrsfTelemetry::update(
    const TelemetryState &state,
    uint32_t nowMs)
{
    // --------------------------------------------------------
    // Do not overwrite a frame that has not been transmitted.
    // --------------------------------------------------------

    if (_hasFrame)
    {
        return;
    }

    // --------------------------------------------------------
    // Battery
    // --------------------------------------------------------

    if ((nowMs - _lastBatteryMs) >=
        BatteryIntervalMs)
    {
        if (buildBatteryFrame(state))
        {
            _lastBatteryMs = nowMs;
            return;
        }
    }

    // --------------------------------------------------------
    // GPS
    // --------------------------------------------------------

    if ((nowMs - _lastGpsMs) >=
        GpsIntervalMs)
    {
        if (buildGpsFrame(state))
        {
            _lastGpsMs = nowMs;
            return;
        }
    }

    // --------------------------------------------------------
    // Variometer
    // --------------------------------------------------------

    if ((nowMs - _lastVarioMs) >=
        VarioIntervalMs)
    {
        if (buildVarioFrame(state))
        {
            _lastVarioMs = nowMs;
            return;
        }
    }

    // --------------------------------------------------------
    // Attitude
    // --------------------------------------------------------

    if ((nowMs - _lastAttitudeMs) >=
        AttitudeIntervalMs)
    {
        if (buildAttitudeFrame(state))
        {
            _lastAttitudeMs = nowMs;
            return;
        }
    }

    // --------------------------------------------------------
    // Flight mode
    // --------------------------------------------------------

    if ((nowMs - _lastFlightModeMs) >=
        FlightModeIntervalMs)
    {
        if (buildFlightModeFrame(state))
        {
            _lastFlightModeMs = nowMs;
            return;
        }
    }
}

// ============================================================================
// Battery
// ============================================================================

bool CrsfTelemetry::buildBatteryFrame(
    const TelemetryState &state)
{
    if (!beginFrame(
            TypeBattery,
            BatteryPayloadSize))
    {
        return false;
    }

    /*
     * CRSF Battery Sensor
     *
     * Payload:
     *
     * uint16 voltage
     * uint16 current
     * uint24 capacity
     * uint8  remaining
     *
     * CRSF units:
     *
     * voltage:
     *     0.01 mV? No.
     *
     * Common CRSF implementation:
     *
     *     voltage = mV * 100
     *     current = mA * 100
     *
     * Therefore:
     *
     *     volts * 100000
     *     amps  * 100000
     */

    float voltage = state.batteryVoltage;

    if (!std::isfinite(voltage) ||
        voltage < 0.0f)
    {
        voltage = 0.0f;
    }

    float current = state.batteryCurrent;

    if (!std::isfinite(current) ||
        current < 0.0f)
    {
        current = 0.0f;
    }

    float consumed = state.batteryConsumedMah;

    if (!std::isfinite(consumed) ||
        consumed < 0.0f)
    {
        consumed = 0.0f;
    }

    const uint16_t encodedVoltage =
        static_cast<uint16_t>(
            std::clamp(
                voltage * 100000.0f,
                0.0f,
                65535.0f));

    const uint16_t encodedCurrent =
        static_cast<uint16_t>(
            std::clamp(
                current * 100000.0f,
                0.0f,
                65535.0f));

    const uint32_t encodedCapacity =
        static_cast<uint32_t>(
            std::clamp(
                consumed,
                0.0f,
                16777215.0f));

    writeU16(
        &_buffer[3],
        encodedVoltage);

    writeU16(
        &_buffer[5],
        encodedCurrent);

    writeU24(
        &_buffer[7],
        encodedCapacity);

    _buffer[10] =
        std::min<uint8_t>(
            state.batteryRemaining,
            100U);

    /*
     * CRC:
     *
     * type + payload
     *
     * 1 + 8 = 9 bytes
     */

    _buffer[11] =
        CrsfCrc::calculate(
            &_buffer[2],
            9);

    _frameSize = 12;

    _hasFrame = true;

    return true;
}

// ============================================================================
// GPS
// ============================================================================

bool CrsfTelemetry::buildGpsFrame(
    const TelemetryState &state)
{
    if (!beginFrame(
            TypeGps,
            GpsPayloadSize))
    {
        return false;
    }

    /*
     * CRSF GPS
     *
     * int32 latitude
     * int32 longitude
     * uint16 ground speed
     * uint16 heading
     * uint16 altitude
     * uint8 satellites
     *
     * Units:
     *
     * latitude:
     *     degrees * 10,000,000
     *
     * longitude:
     *     degrees * 10,000,000
     *
     * ground speed:
     *     km/h * 10
     *
     * heading:
     *     degrees * 100
     *
     * altitude:
     *     meters + 1000
     */

    float latitude =
        static_cast<float>(
            state.latitude);

    float longitude =
        static_cast<float>(
            state.longitude);

    if (!std::isfinite(latitude))
    {
        latitude = 0.0f;
    }

    if (!std::isfinite(longitude))
    {
        longitude = 0.0f;
    }

    latitude =
        std::clamp(
            latitude,
            -180.0f,
            180.0f);

    longitude =
        std::clamp(
            longitude,
            -180.0f,
            180.0f);

    const int32_t encodedLatitude =
        static_cast<int32_t>(
            latitude * 10000000.0f);

    const int32_t encodedLongitude =
        static_cast<int32_t>(
            longitude * 10000000.0f);

    // --------------------------------------------------------
    // Ground speed
    //
    // TelemetryState::groundSpeed is assumed to be m/s.
    //
    // m/s -> km/h:
    //
    // m/s * 3.6
    //
    // CRSF uses km/h * 10:
    //
    // m/s * 36
    // --------------------------------------------------------

    float groundSpeed =
        state.groundSpeed;

    if (!std::isfinite(groundSpeed) ||
        groundSpeed < 0.0f)
    {
        groundSpeed = 0.0f;
    }

    const uint16_t encodedGroundSpeed =
        static_cast<uint16_t>(
            std::clamp(
                groundSpeed * 36.0f,
                0.0f,
                65535.0f));

    // --------------------------------------------------------
    // Heading
    // --------------------------------------------------------

    float heading =
        state.gpsHeading;

    if (!std::isfinite(heading))
    {
        heading = 0.0f;
    }

    heading =
        std::fmod(
            heading,
            360.0f);

    if (heading < 0.0f)
    {
        heading += 360.0f;
    }

    const uint16_t encodedHeading =
        static_cast<uint16_t>(
            std::clamp(
                heading * 100.0f,
                0.0f,
                65535.0f));

    // --------------------------------------------------------
    // Altitude
    //
    // CRSF GPS altitude uses +1000 m offset.
    // --------------------------------------------------------

    float altitude =
        state.gpsAltitude;

    if (!std::isfinite(altitude))
    {
        altitude = 0.0f;
    }

    altitude += 1000.0f;

    const uint16_t encodedAltitude =
        static_cast<uint16_t>(
            std::clamp(
                altitude,
                0.0f,
                65535.0f));

    // --------------------------------------------------------
    // Serialize
    // --------------------------------------------------------

    writeI32(
        &_buffer[3],
        encodedLatitude);

    writeI32(
        &_buffer[7],
        encodedLongitude);

    writeU16(
        &_buffer[11],
        encodedGroundSpeed);

    writeU16(
        &_buffer[13],
        encodedHeading);

    writeU16(
        &_buffer[15],
        encodedAltitude);

    _buffer[17] =
        state.satellites;

    // --------------------------------------------------------
    // CRC
    //
    // type + 15 byte payload
    // = 16 bytes
    // --------------------------------------------------------

    _buffer[18] =
        CrsfCrc::calculate(
            &_buffer[2],
            16);

    _frameSize = 19;

    _hasFrame = true;

    return true;
}

// ============================================================================
// Variometer
// ============================================================================

bool CrsfTelemetry::buildVarioFrame(
    const TelemetryState &state)
{
    if (!beginFrame(
            TypeVario,
            VarioPayloadSize))
    {
        return false;
    }

    /*
     * CRSF Vario:
     *
     * int16 vertical speed
     *
     * Unit:
     *
     * cm/s
     */

    float verticalSpeed =
        state.verticalSpeed;

    if (!std::isfinite(verticalSpeed))
    {
        verticalSpeed = 0.0f;
    }

    const float cmPerSecond =
        verticalSpeed * 100.0f;

    const int16_t encoded =
        static_cast<int16_t>(
            std::clamp(
                cmPerSecond,
                -32768.0f,
                32767.0f));

    writeI16(
        &_buffer[3],
        encoded);

    // type + payload = 3 bytes

    _buffer[5] =
        CrsfCrc::calculate(
            &_buffer[2],
            3);

    _frameSize = 6;

    _hasFrame = true;

    return true;
}

// ============================================================================
// Attitude
// ============================================================================

bool CrsfTelemetry::buildAttitudeFrame(
    const TelemetryState &state)
{
    if (!beginFrame(
            TypeAttitude,
            AttitudePayloadSize))
    {
        return false;
    }

    /*
     * CRSF Attitude:
     *
     * int16 pitch
     * int16 roll
     * int16 yaw
     *
     * Unit:
     *
     * radians * 10000
     */

    constexpr float DegreesToRadians =
        0.01745329251994329577f;

    constexpr float Scale =
        10000.0f;

    float pitch = state.pitch;
    float roll = state.roll;
    float yaw = state.yaw;

    if (!std::isfinite(pitch))
    {
        pitch = 0.0f;
    }

    if (!std::isfinite(roll))
    {
        roll = 0.0f;
    }

    if (!std::isfinite(yaw))
    {
        yaw = 0.0f;
    }

    const int16_t encodedPitch =
        static_cast<int16_t>(
            std::clamp(
                pitch *
                    DegreesToRadians *
                    Scale,
                -32768.0f,
                32767.0f));

    const int16_t encodedRoll =
        static_cast<int16_t>(
            std::clamp(
                roll *
                    DegreesToRadians *
                    Scale,
                -32768.0f,
                32767.0f));

    const int16_t encodedYaw =
        static_cast<int16_t>(
            std::clamp(
                yaw *
                    DegreesToRadians *
                    Scale,
                -32768.0f,
                32767.0f));

    writeI16(
        &_buffer[3],
        encodedPitch);

    writeI16(
        &_buffer[5],
        encodedRoll);

    writeI16(
        &_buffer[7],
        encodedYaw);

    // type + 6 byte payload

    _buffer[9] =
        CrsfCrc::calculate(
            &_buffer[2],
            7);

    _frameSize = 10;

    _hasFrame = true;

    return true;
}

// ============================================================================
// Flight Mode
// ============================================================================

bool CrsfTelemetry::buildFlightModeFrame(
    const TelemetryState &state)
{
    const char *mode = "UNKNOWN";

    switch (state.flightMode)
    {
    case 0:
        mode = "ACRO";
        break;

    case 1:
        mode = "ANGLE";
        break;

    case 2:
        mode = "HORIZON";
        break;

    case 3:
        mode = "GPS";
        break;

    default:
        mode = "UNKNOWN";
        break;
    }

    const size_t modeLength =
        std::strlen(mode);

    /*
     * Payload:
     *
     * mode string + '\0'
     */

    const uint8_t payloadSize =
        static_cast<uint8_t>(
            modeLength + 1);

    if (payloadSize >
        MaxFlightModeLength)
    {
        return false;
    }

    if (!beginFrame(
            TypeFlightMode,
            payloadSize))
    {
        return false;
    }

    // --------------------------------------------------------
    // Copy mode string
    // --------------------------------------------------------

    std::memcpy(
        &_buffer[3],
        mode,
        modeLength);

    // Null termination

    _buffer[3 + modeLength] = '\0';

    // --------------------------------------------------------
    // CRC
    //
    // type + payload
    // --------------------------------------------------------

    const uint8_t crcLength =
        static_cast<uint8_t>(
            1 + payloadSize);

    _buffer[4 + modeLength] =
        CrsfCrc::calculate(
            &_buffer[2],
            crcLength);

    _frameSize =
        static_cast<uint8_t>(
            5 + modeLength);

    _hasFrame = true;

    return true;
}

// ============================================================================
// Begin Frame
// ============================================================================

bool CrsfTelemetry::beginFrame(
    uint8_t type,
    uint8_t payloadSize)
{
    /*
     * CRSF:
     *
     * [address]
     * [length]
     * [type]
     * [payload]
     * [crc]
     *
     * Length =
     *
     * type + payload + crc
     */

    const uint16_t frameLength =
        static_cast<uint16_t>(
            1 +
            payloadSize +
            1);

    const uint16_t totalSize =
        static_cast<uint16_t>(
            frameLength + 2);

    if (totalSize > BufferSize)
    {
        return false;
    }

    _buffer[0] =
        AddressReceiver;

    _buffer[1] =
        static_cast<uint8_t>(
            frameLength);

    _buffer[2] =
        type;

    _frameSize = 0;

    return true;
}

// ============================================================================
// Serialization - uint16
// ============================================================================

void CrsfTelemetry::writeU16(
    uint8_t *buffer,
    uint16_t value)
{
    buffer[0] =
        static_cast<uint8_t>(
            value >> 8);

    buffer[1] =
        static_cast<uint8_t>(
            value & 0xFF);
}

// ============================================================================
// Serialization - int16
// ============================================================================

void CrsfTelemetry::writeI16(
    uint8_t *buffer,
    int16_t value)
{
    writeU16(
        buffer,
        static_cast<uint16_t>(
            value));
}

// ============================================================================
// Serialization - uint24
// ============================================================================

void CrsfTelemetry::writeU24(
    uint8_t *buffer,
    uint32_t value)
{
    buffer[0] =
        static_cast<uint8_t>(
            (value >> 16) & 0xFF);

    buffer[1] =
        static_cast<uint8_t>(
            (value >> 8) & 0xFF);

    buffer[2] =
        static_cast<uint8_t>(
            value & 0xFF);
}

// ============================================================================
// Serialization - int32
// ============================================================================

void CrsfTelemetry::writeI32(
    uint8_t *buffer,
    int32_t value)
{
    writeU32(
        buffer,
        static_cast<uint32_t>(
            value));
}

// ============================================================================
// Serialization - uint32
// ============================================================================

void CrsfTelemetry::writeU32(
    uint8_t *buffer,
    uint32_t value)
{
    buffer[0] =
        static_cast<uint8_t>(
            (value >> 24) & 0xFF);

    buffer[1] =
        static_cast<uint8_t>(
            (value >> 16) & 0xFF);

    buffer[2] =
        static_cast<uint8_t>(
            (value >> 8) & 0xFF);

    buffer[3] =
        static_cast<uint8_t>(
            value & 0xFF);
}

// ============================================================================
// Frame state
// ============================================================================

bool CrsfTelemetry::hasFrame() const
{
    return _hasFrame;
}

const uint8_t *
CrsfTelemetry::frame() const
{
    return _buffer;
}

uint8_t CrsfTelemetry::frameSize() const
{
    return _frameSize;
}

// ============================================================================
// Clear frame
// ============================================================================

void CrsfTelemetry::clearFrame()
{
    _hasFrame = false;
    _frameSize = 0;
}