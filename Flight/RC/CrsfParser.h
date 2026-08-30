#pragma once

#include <cstdint>

/**
 * @brief CRSF RC channel frame parser.
 *
 * Parses CRSF RC_CHANNELS_PACKED frames and extracts
 * all 16 channels encoded as 11-bit values.
 *
 * Native CRSF channel range is approximately:
 *
 *     172 ... 1811
 *
 * The parser does not perform:
 *
 *     - channel normalization
 *     - channel mapping
 *     - switch interpretation
 *     - failsafe handling
 *
 * Those responsibilities belong to RCInput.
 */
class CrsfParser
{
public:
    static constexpr uint8_t ChannelCount = 16;

    /**
     * @brief Construct a CRSF parser.
     */
    CrsfParser();

    /**
     * @brief Process one byte from the CRSF stream.
     *
     * @param byte Incoming CRSF byte.
     *
     * @return true when a valid RC_CHANNELS_PACKED
     *         frame has been decoded.
     */
    bool processByte(
        uint8_t byte);

    /**
     * @brief Get the latest decoded RC channels.
     *
     * @return Pointer to an array containing 16
     *         raw CRSF channel values.
     */
    const uint16_t *channels() const;

    /**
     * @brief Check whether a new valid RC frame is available.
     *
     * @return true if a new frame has been decoded.
     */
    bool hasNewFrame() const;

    /**
     * @brief Clear the new-frame flag.
     */
    void clearNewFrame();

private:
    /**
     * @brief Decode the currently buffered frame.
     */
    bool decodeFrame();

    /**
     * @brief Validate the CRC of the current frame.
     */
    bool validateCrc() const;

    /**
     * @brief Reset the frame reception state.
     *
     * Does not clear the decoded channels or
     * the new-frame flag.
     */
    void reset();

private:
    // ------------------------------------------------------------------------
    // CRSF frame limits
    // ------------------------------------------------------------------------

    static constexpr uint8_t BufferSize = 64;

    // ------------------------------------------------------------------------
    // CRSF protocol
    // ------------------------------------------------------------------------

    static constexpr uint8_t AddressFlightController = 0xC8;

    static constexpr uint8_t TypeRcChannelsPacked = 0x16;

    // ------------------------------------------------------------------------
    // RC_CHANNELS_PACKED
    //
    // type    = 1 byte
    // payload = 22 bytes
    // CRC     = 1 byte
    //
    // length = 24 bytes
    // ------------------------------------------------------------------------

    static constexpr uint8_t RcPayloadSize = 22;

    static constexpr uint8_t RcFrameLength = 24;

    // ------------------------------------------------------------------------
    // Reception buffer
    // ------------------------------------------------------------------------

    uint8_t _buffer[BufferSize]{};

    uint8_t _index = 0;

    uint8_t _frameLength = 0;

    // ------------------------------------------------------------------------
    // Decoded channels
    // ------------------------------------------------------------------------

    uint16_t _channels[ChannelCount]{};

    // ------------------------------------------------------------------------
    // Frame state
    // ------------------------------------------------------------------------

    bool _newFrame = false;
};