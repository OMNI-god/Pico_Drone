#pragma once

#include <cstdint>

/**
 * @brief CRSF CRC8 utility.
 *
 * Implements the CRC8 algorithm used by the
 * Crossfire / ExpressLRS CRSF protocol.
 *
 * Polynomial:
 *     0xD5
 *
 * Initial value:
 *     0x00
 *
 * No reflection.
 *
 * No final XOR.
 */
class CrsfCrc
{
public:
    /**
     * @brief Calculate CRSF CRC8.
     *
     * @param data   Pointer to the data buffer.
     * @param length Number of bytes to process.
     *
     * @return Calculated CRC8 value.
     *
     * @note
     * The caller is responsible for passing the
     * correct CRSF CRC-covered bytes.
     */
    static uint8_t calculate(
        const uint8_t *data,
        uint8_t length);

private:
    CrsfCrc() = delete;
};