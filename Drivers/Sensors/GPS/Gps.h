#pragma once

#include <cstdint>

#include "hardware/uart.h"

/*
 * GPS receiver driver for the ADH-Tech GP1513 / MC-1513
 * (MediaTek MT3339 based) module.
 *
 * The module speaks NMEA 0183 over its UART interface:
 *
 *     9600 baud, 8 data bits, no parity, 1 stop bit
 *
 * Default sentences:
 *
 *     GGA - position, altitude, satellites, fix quality
 *     GLL - latitude / longitude
 *     GSA - GNSS fix type
 *     GSV - satellites in view
 *     RMC - position, ground speed, course, status
 *     VTG - course over ground and ground speed
 *
 * The driver accumulates bytes into complete sentences,
 * validates the NMEA checksum and decodes GGA / RMC /
 * VTG / GSA into GpsState.
 */
class Gps
{
public:
    struct GpsState
    {
        /*
         * True when the receiver currently reports a
         * valid position fix.
         */
        bool valid = false;

        /*
         * True when the receiver reports a 3D fix.
         */
        bool hasFix = false;

        /*
         * Latitude in decimal degrees.
         */
        double latitude = 0.0;

        /*
         * Longitude in decimal degrees.
         */
        double longitude = 0.0;

        /*
         * Altitude in meters.
         */
        float altitudeM = 0.0f;

        /*
         * Ground speed in meters per second.
         */
        float groundSpeedMs = 0.0f;

        /*
         * Course over ground in degrees.
         *
         * Range:
         *
         *     0 ... <360
         */
        float headingDeg = 0.0f;

        /*
         * Number of satellites used for the fix.
         */
        std::uint8_t satellites = 0;

        /*
         * Timestamp of the most recent valid fix
         * sentence, in microseconds.
         */
        std::uint64_t timestampUs = 0;
    };

    Gps(
        uart_inst_t *uart,
        std::uint32_t baudrate,
        std::uint32_t rxPin,
        std::uint32_t txPin);

    bool initialize();

    bool isInitialized() const;

    /*
     * Read every pending byte from the UART and parse
     * complete NMEA sentences.
     *
     * Returns true when at least one valid fix sentence
     * was decoded during this call.
     */
    bool read(
        GpsState &state);

    const GpsState &getState() const;

    /*
     * True when the receiver currently reports a
     * valid position fix.
     */
    bool isValid() const;

    /*
     * True when the most recent fix is not older than
     * maxAgeUs.
     */
    bool isFresh(
        std::uint64_t nowUs,
        std::uint64_t maxAgeUs) const;

    /*
     * Number of bytes received since initialization.
     *
     * Useful for diagnosing wiring problems.
     */
    std::uint32_t getReceivedBytes() const;

private:
    // =====================================================================
    // NMEA reception
    // =====================================================================

    static constexpr std::uint16_t SentenceSize = 128;

    static constexpr std::uint8_t MaxFields = 24;

    /*
     * Feed one byte into the sentence assembler.
     *
     * Returns true when a complete line has been
     * assembled and should be parsed.
     */
    bool feedByte(
        char byte);

    /*
     * Parse one complete NMEA sentence.
     *
     * Returns true when the sentence was valid and
     * updated the GPS state.
     */
    bool parseSentence(
        const char *sentence,
        std::uint16_t length);

    /*
     * Validate the NMEA checksum.
     */
    static bool verifyChecksum(
        const char *sentence,
        std::uint16_t length);

    /*
     * Split a sentence into comma separated fields.
     */
    static std::uint8_t splitFields(
        char *sentence,
        char *fields[],
        std::uint8_t maxFields);

    /*
     * Decode a decimal coordinate in the NMEA
     * "ddmm.mmmm" / "dddmm.mmmm" format.
     */
    static double decodeCoordinate(
        const char *text,
        const char *hemisphere);

    /*
     * Convert knots to meters per second.
     */
    static float knotsToMetersPerSecond(
        float knots);

    /*
     * Convert kilometers per hour to meters per second.
     */
    static float kilometersPerHourToMetersPerSecond(
        float kmh);

    // =====================================================================
    // UART
    // =====================================================================

    uart_inst_t *uartInstance_;

    std::uint32_t baudrate_;

    std::uint32_t rxPin_;

    std::uint32_t txPin_;

    bool initialized_ = false;

    // =====================================================================
    // Sentence assembler
    // =====================================================================

    char sentence_[SentenceSize];

    std::uint16_t sentenceIndex_ = 0;

    bool sentenceActive_ = false;

    // =====================================================================
    // State
    // =====================================================================

    GpsState state_;

    std::uint32_t receivedBytes_ = 0;
};
