#include "CrsfCrc.h"

namespace
{
    constexpr uint8_t CRSF_CRC8_POLYNOMIAL = 0xD5;
    constexpr uint8_t BITS_PER_BYTE = 8;
}

uint8_t CrsfCrc::calculate(
    const uint8_t *data,
    uint8_t length)
{
    if (data == nullptr || length == 0)
    {
        return 0;
    }

    uint8_t crc = 0;

    for (uint8_t i = 0;
         i < length;
         ++i)
    {
        crc ^= data[i];

        for (uint8_t bit = 0;
             bit < BITS_PER_BYTE;
             ++bit)
        {
            if ((crc & 0x80u) != 0u)
            {
                crc =
                    static_cast<uint8_t>(
                        (crc << 1u) ^
                        CRSF_CRC8_POLYNOMIAL);
            }
            else
            {
                crc =
                    static_cast<uint8_t>(
                        crc << 1u);
            }
        }
    }

    return crc;
}