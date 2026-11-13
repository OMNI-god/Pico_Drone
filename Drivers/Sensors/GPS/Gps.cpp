#include "Gps.h"

#include "PioUartRx.h"

#include "pico/stdlib.h"
#include "pico/time.h"

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>

// =============================================================================
// Constructor
// =============================================================================

Gps::Gps(
    uart_inst_t *uart,
    std::uint32_t baudrate,
    std::uint32_t rxPin,
    std::uint32_t txPin)
    : uartInstance_(uart),
      baudrate_(baudrate),
      rxPin_(rxPin),
      txPin_(txPin)
{
}

Gps::Gps(
    PioUartRx &pioRx,
    std::uint32_t baudrate,
    std::uint32_t txPin)
    : uartInstance_(nullptr),
      pioRx_(&pioRx),
      baudrate_(baudrate),
      rxPin_(pioRx.getRxPin()),
      txPin_(txPin)
{
}

// =============================================================================
// Initialize
// =============================================================================

bool Gps::initialize()
{
    if (initialized_)
    {
        return true;
    }

    /*
     * PIO receive path.
     *
     * Used when the hardware UART is already taken
     * by the ELRS/CRSF link.
     */

    if (pioRx_ != nullptr)
    {
        if (!pioRx_->initialize(baudrate_))
        {
            printf("GPS: PIO initialization failed\n");
            return false;
        }

        /*
         * The flight controller never transmits to
         * the GPS, but the module's RX input must
         * not float. Hold it at the UART idle level.
         */

        gpio_init(txPin_);
        gpio_set_dir(txPin_, GPIO_OUT);
        gpio_put(txPin_, 1);

        initialized_ = true;

        printf(
            "GPS: PIO RX initialized (%lu baud, RX GPIO %lu, TX GPIO %lu idle)\n",
            static_cast<unsigned long>(baudrate_),
            static_cast<unsigned long>(rxPin_),
            static_cast<unsigned long>(txPin_));

        return true;
    }

    if (uartInstance_ == nullptr)
    {
        printf("GPS: invalid UART instance\n");
        return false;
    }

    uart_init(
        uartInstance_,
        baudrate_);

    gpio_set_function(
        rxPin_,
        GPIO_FUNC_UART);

    gpio_set_function(
        txPin_,
        GPIO_FUNC_UART);

    initialized_ = true;

    printf(
        "GPS: UART initialized (%lu baud, RX GPIO %lu, TX GPIO %lu)\n",
        static_cast<unsigned long>(baudrate_),
        static_cast<unsigned long>(rxPin_),
        static_cast<unsigned long>(txPin_));

    return true;
}

// =============================================================================
// Status
// =============================================================================

bool Gps::isInitialized() const
{
    return initialized_;
}

bool Gps::isValid() const
{
    return state_.valid;
}

bool Gps::isFresh(
    std::uint64_t nowUs,
    std::uint64_t maxAgeUs) const
{
    if (!state_.valid ||
        state_.timestampUs == 0)
    {
        return false;
    }

    if (nowUs < state_.timestampUs)
    {
        return false;
    }

    return (nowUs - state_.timestampUs) <= maxAgeUs;
}

std::uint32_t Gps::getReceivedBytes() const
{
    return receivedBytes_;
}

const Gps::GpsState &
Gps::getState() const
{
    return state_;
}

// =============================================================================
// Read
// =============================================================================

bool Gps::read(
    GpsState &state)
{
    bool decodedFix = false;

    if (!initialized_)
    {
        state = state_;
        return false;
    }

    /*
     * Drain the UART receive FIFO.
     *
     * The module streams NMEA sentences continuously,
     * so every pending byte is consumed here.
     */

    if (pioRx_ != nullptr)
    {
        while (pioRx_->available())
        {
            const int value = pioRx_->read();

            if (value < 0)
            {
                break;
            }

            const char byte =
                static_cast<char>(value);

            ++receivedBytes_;

            if (feedByte(byte) &&
                parseSentence(
                    sentence_,
                    sentenceIndex_))
            {
                decodedFix = true;
            }
        }

        state = state_;

        return decodedFix;
    }

    while (uart_is_readable(uartInstance_))
    {
        const char byte =
            static_cast<char>(
                uart_getc(uartInstance_));

        ++receivedBytes_;

        if (feedByte(byte) &&
            parseSentence(
                sentence_,
                sentenceIndex_))
        {
            decodedFix = true;
        }
    }

    state = state_;

    return decodedFix;
}

// =============================================================================
// Feed byte
// =============================================================================

bool Gps::feedByte(
    char byte)
{
    /*
     * A new sentence always starts with '$'.
     */

    if (byte == '$')
    {
        sentenceIndex_ = 0;
        sentenceActive_ = true;

        if (sentenceIndex_ < SentenceSize)
        {
            sentence_[sentenceIndex_++] = byte;
        }

        return false;
    }

    if (!sentenceActive_)
    {
        return false;
    }

    if (sentenceIndex_ < SentenceSize)
    {
        sentence_[sentenceIndex_++] = byte;
    }

    /*
     * Sentences are terminated by LF.
     */

    if (byte == '\n' ||
        sentenceIndex_ >= SentenceSize)
    {
        const bool complete =
            byte == '\n' &&
            sentenceIndex_ < SentenceSize;

        sentenceActive_ = false;

        return complete;
    }

    return false;
}

// =============================================================================
// Parse sentence
// =============================================================================

bool Gps::parseSentence(
    const char *sentence,
    std::uint16_t length)
{
    if (sentence == nullptr ||
        length < 8)
    {
        return false;
    }

    if (sentence[0] != '$')
    {
        return false;
    }

    if (!verifyChecksum(
            sentence,
            length))
    {
        return false;
    }

    /*
     * Work on a mutable copy so that fields can be
     * split in place.
     */

    char buffer[SentenceSize];

    const std::uint16_t copyLength =
        length < SentenceSize ?
            length :
            SentenceSize - 1;

    std::memcpy(
        buffer,
        sentence,
        copyLength);

    buffer[copyLength] = '\0';

    char *fields[MaxFields] = {};

    const std::uint8_t fieldCount =
        splitFields(
            buffer,
            fields,
            MaxFields);

    if (fieldCount < 2 ||
        fields[0] == nullptr ||
        std::strlen(fields[0]) < 6)
    {
        return false;
    }

    /*
     * The sentence identifier is the last three
     * characters of the first field:
     *
     *     "$GPGGA" -> "GGA"
     *     "$GNRMC" -> "RMC"
     */

    const char *identifier =
        &fields[0][
            std::strlen(fields[0]) - 3];

    bool updated = false;

    if (std::strcmp(identifier, "GGA") == 0)
    {
        updated = true;

        /*
         * $GPGGA,time,lat,N/S,lon,E/W,quality,sats,hdop,alt,M,...
         */

        const int quality =
            (fieldCount > 6 && fields[6] != nullptr) ?
                std::atoi(fields[6]) :
                0;

        if (quality >= 1 &&
            fieldCount > 9 &&
            fields[2] != nullptr &&
            fields[4] != nullptr)
        {
            state_.latitude =
                decodeCoordinate(
                    fields[2],
                    fields[3]);

            state_.longitude =
                decodeCoordinate(
                    fields[4],
                    fields[5]);

            state_.valid = true;
            state_.hasFix = true;

            state_.timestampUs =
                time_us_64();
        }
        else
        {
            state_.valid = false;
        }

        if (fieldCount > 7 &&
            fields[7] != nullptr)
        {
            state_.satellites =
                static_cast<std::uint8_t>(
                    std::atoi(fields[7]));
        }

        if (fieldCount > 9 &&
            fields[9] != nullptr &&
            fields[9][0] != '\0')
        {
            state_.altitudeM =
                static_cast<float>(
                    std::atof(fields[9]));
        }
    }
    else if (std::strcmp(identifier, "RMC") == 0)
    {
        updated = true;

        /*
         * $GPRMC,time,status,lat,N/S,lon,E/W,speed,course,date,...
         */

        const bool statusValid =
            fieldCount > 2 &&
            fields[2] != nullptr &&
            fields[2][0] == 'A';

        if (statusValid &&
            fieldCount > 6 &&
            fields[3] != nullptr &&
            fields[5] != nullptr)
        {
            state_.latitude =
                decodeCoordinate(
                    fields[3],
                    fields[4]);

            state_.longitude =
                decodeCoordinate(
                    fields[5],
                    fields[6]);

            state_.valid = true;
            state_.hasFix = true;

            state_.timestampUs =
                time_us_64();
        }
        else if (!statusValid)
        {
            state_.valid = false;
        }

        if (fieldCount > 7 &&
            fields[7] != nullptr &&
            fields[7][0] != '\0')
        {
            state_.groundSpeedMs =
                knotsToMetersPerSecond(
                    static_cast<float>(
                        std::atof(fields[7])));
        }

        if (fieldCount > 8 &&
            fields[8] != nullptr &&
            fields[8][0] != '\0')
        {
            state_.headingDeg =
                static_cast<float>(
                    std::atof(fields[8]));
        }
    }
    else if (std::strcmp(identifier, "VTG") == 0)
    {
        updated = true;

        /*
         * $GPVTG,course,T,course,M,speed,N,speed,K,mode
         */

        if (fieldCount > 1 &&
            fields[1] != nullptr &&
            fields[1][0] != '\0')
        {
            const float course =
                static_cast<float>(
                    std::atof(fields[1]));

            if (course >= 0.0f &&
                course < 360.0f)
            {
                state_.headingDeg = course;
            }
        }

        if (fieldCount > 7 &&
            fields[7] != nullptr &&
            fields[7][0] != '\0')
        {
            state_.groundSpeedMs =
                kilometersPerHourToMetersPerSecond(
                    static_cast<float>(
                        std::atof(fields[7])));
        }
        else if (fieldCount > 5 &&
                 fields[5] != nullptr &&
                 fields[5][0] != '\0')
        {
            state_.groundSpeedMs =
                knotsToMetersPerSecond(
                    static_cast<float>(
                        std::atof(fields[5])));
        }
    }
    else if (std::strcmp(identifier, "GSA") == 0)
    {
        updated = true;

        /*
         * $GPGSA,mode,fixtype,sv,...,pdop,hdop,vdop
         */

        if (fieldCount > 2 &&
            fields[2] != nullptr)
        {
            const int fixType =
                std::atoi(fields[2]);

            state_.hasFix = fixType >= 3;

            if (fixType < 2)
            {
                state_.valid = false;
            }
        }
    }

    return updated;
}

// =============================================================================
// Checksum
// =============================================================================

bool Gps::verifyChecksum(
    const char *sentence,
    std::uint16_t length)
{
    if (sentence == nullptr ||
        length < 8)
    {
        return false;
    }

    /*
     * Find the checksum delimiter.
     */

    std::uint16_t delimiterIndex = 0;

    for (std::uint16_t i = 1;
         i < length;
         ++i)
    {
        if (sentence[i] == '*')
        {
            delimiterIndex = i;
            break;
        }
    }

    if (delimiterIndex == 0 ||
        delimiterIndex + 3 > length)
    {
        return false;
    }

    /*
     * XOR every character between '$' and '*'.
     */

    std::uint8_t checksum = 0;

    for (std::uint16_t i = 1;
         i < delimiterIndex;
         ++i)
    {
        checksum ^=
            static_cast<std::uint8_t>(
                sentence[i]);
    }

    /*
     * Decode the two hexadecimal digits.
     */

    const char highChar =
        sentence[delimiterIndex + 1];

    const char lowChar =
        sentence[delimiterIndex + 2];

    auto hexValue =
        [](char c) -> int
        {
            if (c >= '0' && c <= '9')
            {
                return c - '0';
            }

            if (c >= 'A' && c <= 'F')
            {
                return c - 'A' + 10;
            }

            if (c >= 'a' && c <= 'f')
            {
                return c - 'a' + 10;
            }

            return -1;
        };

    const int high = hexValue(highChar);
    const int low = hexValue(lowChar);

    if (high < 0 ||
        low < 0)
    {
        return false;
    }

    const std::uint8_t expected =
        static_cast<std::uint8_t>(
            (high << 4) | low);

    return checksum == expected;
}

// =============================================================================
// Split fields
// =============================================================================

std::uint8_t Gps::splitFields(
    char *sentence,
    char *fields[],
    std::uint8_t maxFields)
{
    std::uint8_t count = 0;

    if (sentence == nullptr ||
        fields == nullptr ||
        maxFields == 0)
    {
        return 0;
    }

    fields[count++] = sentence;

    for (char *cursor = sentence;
         *cursor != '\0';
         ++cursor)
    {
        if (*cursor == ',')
        {
            *cursor = '\0';

            if (count < maxFields)
            {
                fields[count++] =
                    cursor + 1;
            }
        }
        else if (*cursor == '*')
        {
            /*
             * Remove the checksum from the last field.
             */

            *cursor = '\0';
            break;
        }
    }

    return count;
}

// =============================================================================
// Coordinate conversion
// =============================================================================

double Gps::decodeCoordinate(
    const char *text,
    const char *hemisphere)
{
    if (text == nullptr ||
        text[0] == '\0')
    {
        return 0.0;
    }

    const double value =
        std::atof(text);

    /*
     * NMEA coordinates use the format:
     *
     *     ddmm.mmmm   (latitude)
     *     dddmm.mmmm  (longitude)
     *
     * Therefore:
     *
     *     degrees = floor(value / 100)
     *     minutes = value - degrees * 100
     */

    const double degrees =
        std::floor(value / 100.0);

    const double minutes =
        value - degrees * 100.0;

    double decimal =
        degrees + minutes / 60.0;

    if (hemisphere != nullptr)
    {
        if (hemisphere[0] == 'S' ||
            hemisphere[0] == 'W')
        {
            decimal = -decimal;
        }
    }

    return decimal;
}

// =============================================================================
// Unit conversion
// =============================================================================

float Gps::knotsToMetersPerSecond(
    float knots)
{
    return knots * 0.514444f;
}

float Gps::kilometersPerHourToMetersPerSecond(
    float kmh)
{
    return kmh / 3.6f;
}
