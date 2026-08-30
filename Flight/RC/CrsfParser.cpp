#include "CrsfParser.h"

#include "CrsfCrc.h"

CrsfParser::CrsfParser()
{
    reset();
}

bool CrsfParser::processByte(
    uint8_t byte)
{
    // ------------------------------------------------------------------------
    // Wait for CRSF address / sync byte
    // ------------------------------------------------------------------------

    if (_index == 0)
    {
        if (byte != AddressFlightController)
        {
            return false;
        }

        _buffer[_index++] = byte;

        return false;
    }

    // ------------------------------------------------------------------------
    // Prevent buffer overflow
    // ------------------------------------------------------------------------

    if (_index >= BufferSize)
    {
        reset();
        return false;
    }

    _buffer[_index++] = byte;

    // ------------------------------------------------------------------------
    // Length byte
    //
    // Frame format:
    //
    // [address]
    // [length]
    // [type]
    // [payload]
    // [CRC]
    //
    // The length field represents:
    //
    // type + payload + CRC
    // ------------------------------------------------------------------------

    if (_index == 2)
    {
        _frameLength = byte;

        // Minimum valid CRSF frame:
        //
        // type + CRC = 2 bytes
        //
        // Maximum must fit inside our buffer.
        if (_frameLength < 2 ||
            _frameLength >
                static_cast<uint8_t>(
                    BufferSize - 2))
        {
            reset();
            return false;
        }

        return false;
    }

    // ------------------------------------------------------------------------
    // Check for complete frame
    // ------------------------------------------------------------------------

    const uint16_t expectedFrameSize =
        static_cast<uint16_t>(
            _frameLength) +
        2u;

    if (_index == expectedFrameSize)
    {
        const bool valid =
            decodeFrame();

        // Reset reception state.
        //
        // IMPORTANT:
        // reset() does NOT clear _newFrame.
        reset();

        return valid;
    }

    return false;
}

bool CrsfParser::decodeFrame()
{
    // ------------------------------------------------------------------------
    // Verify frame type
    // ------------------------------------------------------------------------

    if (_buffer[2] !=
        TypeRcChannelsPacked)
    {
        return false;
    }

    // ------------------------------------------------------------------------
    // RC_CHANNELS_PACKED frame size
    //
    // Type    = 1 byte
    // Payload = 22 bytes
    // CRC     = 1 byte
    //
    // Length = 24
    // ------------------------------------------------------------------------

    if (_frameLength !=
        RcFrameLength)
    {
        return false;
    }

    // ------------------------------------------------------------------------
    // Validate CRC
    //
    // CRC covers:
    //
    // [type]
    // [payload]
    //
    // It does NOT cover:
    //
    // [address]
    // [length]
    // [CRC]
    // ------------------------------------------------------------------------

    if (!validateCrc())
    {
        return false;
    }

    // ------------------------------------------------------------------------
    // Payload
    //
    // Frame:
    //
    // [0]  address
    // [1]  length
    // [2]  type
    // [3..24] payload
    // [25] CRC
    //
    // Therefore payload starts at index 3.
    // ------------------------------------------------------------------------

    const uint8_t *payload =
        &_buffer[3];

    // ------------------------------------------------------------------------
    // Decode 16 × 11-bit channels
    //
    // Total payload:
    //
    // 16 × 11 = 176 bits
    //         = 22 bytes
    //
    // CRSF packs the channels continuously,
    // least-significant bit first.
    // ------------------------------------------------------------------------

    uint64_t bitBuffer = 0;

    uint8_t bitsAvailable = 0;

    uint8_t channel = 0;

    for (uint8_t i = 0;
         i < RcPayloadSize;
         ++i)
    {
        bitBuffer |=
            static_cast<uint64_t>(
                payload[i])
            << bitsAvailable;

        bitsAvailable += 8;

        // Extract as many complete 11-bit
        // channels as are available.
        while (bitsAvailable >= 11 &&
               channel < ChannelCount)
        {
            _channels[channel] =
                static_cast<uint16_t>(
                    bitBuffer & 0x07FFu);

            bitBuffer >>= 11;

            bitsAvailable -= 11;

            ++channel;
        }
    }

    // ------------------------------------------------------------------------
    // Ensure all 16 channels were decoded
    // ------------------------------------------------------------------------

    if (channel != ChannelCount)
    {
        return false;
    }

    // ------------------------------------------------------------------------
    // A complete valid RC frame is available
    // ------------------------------------------------------------------------

    _newFrame = true;

    return true;
}

bool CrsfParser::validateCrc() const
{
    // ------------------------------------------------------------------------
    // CRSF CRC covers:
    //
    // type + payload
    //
    // _buffer[2]          = type
    // _buffer[3..24]      = payload
    //
    // _frameLength = 24
    //
    // Therefore:
    //
    // CRC input length = 24 - 1 = 23
    //
    // The last byte in the length-defined
    // portion is the received CRC.
    // ------------------------------------------------------------------------

    if (_frameLength < 2)
    {
        return false;
    }

    const uint8_t calculated =
        CrsfCrc::calculate(
            &_buffer[2],
            static_cast<uint8_t>(
                _frameLength - 1));

    // ------------------------------------------------------------------------
    // CRC location:
    //
    // address index = 0
    // length  index = 1
    //
    // frame length counts from type.
    //
    // CRC index = frameLength + 1
    // ------------------------------------------------------------------------

    const uint8_t received =
        _buffer[static_cast<uint8_t>(
            _frameLength + 1)];

    return calculated == received;
}

const uint16_t *
CrsfParser::channels() const
{
    return _channels;
}

bool CrsfParser::hasNewFrame() const
{
    return _newFrame;
}

void CrsfParser::clearNewFrame()
{
    _newFrame = false;
}

void CrsfParser::reset()
{
    // ------------------------------------------------------------------------
    // Reset frame reception state.
    //
    // Do NOT clear:
    //
    //     _channels
    //     _newFrame
    //
    // The caller needs to be able to retrieve the
    // decoded frame after processByte() returns.
    // ------------------------------------------------------------------------

    _index = 0;

    _frameLength = 0;
}