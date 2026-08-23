#include "BMP280.h"

#include "pico/stdlib.h"

#include <cstdio>
#include <cmath>

// =============================================================================
// Constructor
// =============================================================================

BMP280::BMP280(
    II2c &bus,
    uint8_t address,
    float seaLevelHpa)
    : bus(bus),
      address(address),
      seaLevelHpa(seaLevelHpa)
{
}

// =============================================================================
// Initialize
//
// This intentionally follows the Python implementation:
//
// 1. Check CHIP_ID
// 2. Soft reset
// 3. Wait
// 4. Read calibration
// 5. CTRL_MEAS = 0x27
// 6. CONFIG = 0xA0
//
// No forced measurement.
// No manual trigger.
// Sensor remains in NORMAL mode.
// =============================================================================

bool BMP280::initialize()
{
    initialized = false;
    valid = false;

    printf(
        "BMP280: initializing...\n");

    // =========================================================================
    // CHIP ID
    // =========================================================================

    uint8_t id = 0;

    if (!readRegister(
            REG_ID,
            id))
    {
        printf(
            "BMP280: CHIP_ID read failed\n");

        return false;
    }

    printf(
        "BMP280: CHIP_ID = 0x%02X\n",
        id);

    if (id != CHIP_ID)
    {
        printf(
            "BMP280: invalid CHIP_ID\n");

        return false;
    }

    // =========================================================================
    // SOFT RESET
    // =========================================================================

    printf(
        "BMP280: resetting...\n");

    if (!writeRegister(
            REG_RESET,
            RESET_COMMAND))
    {
        printf(
            "BMP280: reset failed\n");

        return false;
    }

    /*
     * Give the sensor enough time after software reset.
     *
     * This is important because calibration/NVM becomes available
     * only after reset processing has completed.
     */

    sleep_ms(10);

    // =========================================================================
    // CALIBRATION
    // =========================================================================

    if (!readCalibration())
    {
        printf(
            "BMP280: calibration read failed\n");

        return false;
    }

    // =========================================================================
    // CTRL_MEAS
    // =========================================================================

    printf(
        "BMP280: writing CTRL_MEAS = 0x%02X\n",
        CTRL_MEAS_NORMAL);

    if (!writeRegister(
            REG_CTRL_MEAS,
            CTRL_MEAS_NORMAL))
    {
        printf(
            "BMP280: CTRL_MEAS write failed\n");

        return false;
    }

    // =========================================================================
    // CONFIG
    // =========================================================================

    printf(
        "BMP280: writing CONFIG = 0x%02X\n",
        CONFIG_VALUE);

    if (!writeRegister(
            REG_CONFIG,
            CONFIG_VALUE))
    {
        printf(
            "BMP280: CONFIG write failed\n");

        return false;
    }

    // =========================================================================
    // Verify configuration
    // =========================================================================

    uint8_t ctrl = 0;
    uint8_t config = 0;

    if (!readRegister(
            REG_CTRL_MEAS,
            ctrl))
    {
        printf(
            "BMP280: CTRL_MEAS verification failed\n");

        return false;
    }

    if (!readRegister(
            REG_CONFIG,
            config))
    {
        printf(
            "BMP280: CONFIG verification failed\n");

        return false;
    }

    printf(
        "BMP280: CONFIG = 0x%02X\n",
        config);

    printf(
        "BMP280: CTRL_MEAS = 0x%02X\n",
        ctrl);

    if (config != CONFIG_VALUE)
    {
        printf(
            "BMP280: CONFIG mismatch\n");

        return false;
    }

    if (ctrl != CTRL_MEAS_NORMAL)
    {
        printf(
            "BMP280: CTRL_MEAS mismatch\n");

        return false;
    }

    initialized = true;

    printf(
        "BMP280: initialization successful\n");

    /*
     * The sensor is now in normal mode.
     *
     * Do NOT trigger a forced measurement here.
     *
     * The Python driver also simply waits for the sensor to
     * produce measurements.
     */

    return true;
}

// =============================================================================
// Read
//
// Equivalent to:
//
// raw = self._read_raw()
//
// followed by:
//
// temperature compensation
// pressure compensation
// altitude calculation
// sanity checking
// =============================================================================

bool BMP280::read(
    Measurements &measurements)
{
    measurements = {};

    if (!initialized)
    {
        printf(
            "BMP280: not initialized\n");

        return false;
    }

    // =========================================================================
    // Read raw data
    // =========================================================================

    int32_t rawTemperature = 0;
    int32_t rawPressure = 0;

    if (!readRaw(
            rawTemperature,
            rawPressure))
    {
        valid = false;

        printf(
            "BMP280 READ FAILED\n");

        return false;
    }

    // =========================================================================
    // BMP280 startup / skipped measurement check
    // =========================================================================

    /*
     * BMP280 returns 0x80000 when a measurement channel is skipped.
     *
     * This is exactly what you were seeing:
     *
     * F7 = 80
     * F8 = 00
     * F9 = 00
     *
     * => pressure = 0x80000
     *
     * and:
     *
     * FA = 80
     * FB = 00
     * FC = 00
     *
     * => temperature = 0x80000
     *
     * Therefore this must NOT be passed to compensation.
     */

    if (rawTemperature == ADC_INVALID ||
        rawPressure == ADC_INVALID)
    {
        valid = false;

        printf(
            "BMP280: measurement not ready\n");

        return false;
    }

    // =========================================================================
    // Temperature compensation
    // =========================================================================

    float temperatureC =
        compensateTemperature(
            rawTemperature);

    // =========================================================================
    // Pressure compensation
    // =========================================================================

    float pressurePa =
        compensatePressure(
            rawPressure);

    // =========================================================================
    // Pressure sanity check
    // =========================================================================

    float pressureHpa =
        pressurePa / 100.0f;

    /*
     * Same range as Python:
     *
     * 300 hPa <= pressure <= 1100 hPa
     */

    if (!std::isfinite(pressureHpa) ||
        pressureHpa < 300.0f ||
        pressureHpa > 1100.0f)
    {
        valid = false;

        printf(
            "BMP280: invalid pressure = %.2f hPa\n",
            pressureHpa);

        return false;
    }

    // =========================================================================
    // Temperature sanity
    // =========================================================================

    if (!std::isfinite(temperatureC) ||
        temperatureC < -40.0f ||
        temperatureC > 85.0f)
    {
        valid = false;

        printf(
            "BMP280: invalid temperature = %.2f C\n",
            temperatureC);

        return false;
    }

    // =========================================================================
    // Altitude
    // =========================================================================

    float altitudeM =
        altitudeFromPressure(
            pressureHpa);

    // =========================================================================
    // Result
    // =========================================================================

    measurements.temperatureC =
        temperatureC;

    measurements.pressurePa =
        pressurePa;

    measurements.pressureHpa =
        pressureHpa;

    measurements.altitudeM =
        altitudeM;

    measurements.valid =
        true;

    valid = true;

    printf(
        "BMP280: T=%.2f C | P=%.2f hPa | Alt=%.2f m\n",
        temperatureC,
        pressureHpa,
        altitudeM);

    return true;
}

// =============================================================================
// Read temperature
// =============================================================================

bool BMP280::readTemperature(
    float &temperatureC)
{
    Measurements measurements{};

    if (!read(measurements))
    {
        return false;
    }

    temperatureC =
        measurements.temperatureC;

    return true;
}

// =============================================================================
// Read pressure
// =============================================================================

bool BMP280::readPressure(
    float &pressurePa)
{
    Measurements measurements{};

    if (!read(measurements))
    {
        return false;
    }

    pressurePa =
        measurements.pressurePa;

    return true;
}

// =============================================================================
// Connection
// =============================================================================

bool BMP280::isConnected()
{
    uint8_t id = 0;

    if (!readRegister(
            REG_ID,
            id))
    {
        return false;
    }

    return id == CHIP_ID;
}

// =============================================================================
// Valid state
// =============================================================================

bool BMP280::isValid() const
{
    return valid;
}

// =============================================================================
// Sea level pressure
// =============================================================================

void BMP280::setSeaLevelPressure(
    float hpa)
{
    if (hpa >= 900.0f &&
        hpa <= 1100.0f)
    {
        seaLevelHpa = hpa;
    }
}

// =============================================================================
// Read calibration
// =============================================================================

bool BMP280::readCalibration()
{
    uint8_t data[24] = {};

    if (!readRegisters(
            REG_CALIB,
            data,
            sizeof(data)))
    {
        return false;
    }

    // =========================================================================
    // Temperature calibration
    // =========================================================================

    dig_T1 =
        static_cast<uint16_t>(
            static_cast<uint16_t>(data[0]) |
            (static_cast<uint16_t>(data[1]) << 8));

    dig_T2 =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[2]) |
            (static_cast<uint16_t>(data[3]) << 8));

    dig_T3 =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[4]) |
            (static_cast<uint16_t>(data[5]) << 8));

    // =========================================================================
    // Pressure calibration
    // =========================================================================

    dig_P1 =
        static_cast<uint16_t>(
            static_cast<uint16_t>(data[6]) |
            (static_cast<uint16_t>(data[7]) << 8));

    dig_P2 =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[8]) |
            (static_cast<uint16_t>(data[9]) << 8));

    dig_P3 =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[10]) |
            (static_cast<uint16_t>(data[11]) << 8));

    dig_P4 =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[12]) |
            (static_cast<uint16_t>(data[13]) << 8));

    dig_P5 =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[14]) |
            (static_cast<uint16_t>(data[15]) << 8));

    dig_P6 =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[16]) |
            (static_cast<uint16_t>(data[17]) << 8));

    dig_P7 =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[18]) |
            (static_cast<uint16_t>(data[19]) << 8));

    dig_P8 =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[20]) |
            (static_cast<uint16_t>(data[21]) << 8));

    dig_P9 =
        static_cast<int16_t>(
            static_cast<uint16_t>(data[22]) |
            (static_cast<uint16_t>(data[23]) << 8));

    // =========================================================================
    // Print calibration
    // =========================================================================

    printf("\n");
    printf("BMP280 CALIBRATION\n");
    printf("------------------\n");

    printf(
        "T1 = %u\n",
        dig_T1);

    printf(
        "T2 = %d\n",
        dig_T2);

    printf(
        "T3 = %d\n",
        dig_T3);

    printf(
        "P1 = %u\n",
        dig_P1);

    printf(
        "P2 = %d\n",
        dig_P2);

    printf(
        "P3 = %d\n",
        dig_P3);

    printf(
        "P4 = %d\n",
        dig_P4);

    printf(
        "P5 = %d\n",
        dig_P5);

    printf(
        "P6 = %d\n",
        dig_P6);

    printf(
        "P7 = %d\n",
        dig_P7);

    printf(
        "P8 = %d\n",
        dig_P8);

    printf(
        "P9 = %d\n",
        dig_P9);

    printf(
        "------------------\n\n");

    // =========================================================================
    // Validate calibration
    // =========================================================================

    if (dig_T1 == 0 ||
        dig_P1 == 0)
    {
        printf(
            "BMP280: invalid calibration\n");

        return false;
    }

    return true;
}

// =============================================================================
// Read raw data
// =============================================================================

bool BMP280::readRaw(
    int32_t &temperature,
    int32_t &pressure)
{
    uint8_t data[6] = {};

    if (!readRegisters(
            REG_DATA,
            data,
            sizeof(data)))
    {
        printf(
            "BMP280: DATA read failed\n");

        return false;
    }

    // =========================================================================
    // Print raw registers
    // =========================================================================

    printf(
        "BMP280 DATA:\n"
        "  F7 = 0x%02X\n"
        "  F8 = 0x%02X\n"
        "  F9 = 0x%02X\n"
        "  FA = 0x%02X\n"
        "  FB = 0x%02X\n"
        "  FC = 0x%02X\n",
        data[0],
        data[1],
        data[2],
        data[3],
        data[4],
        data[5]);

    // =========================================================================
    // Pressure
    // =========================================================================

    pressure =
        (static_cast<int32_t>(data[0]) << 12) |
        (static_cast<int32_t>(data[1]) << 4) |
        (static_cast<int32_t>(data[2]) >> 4);

    // =========================================================================
    // Temperature
    // =========================================================================

    temperature =
        (static_cast<int32_t>(data[3]) << 12) |
        (static_cast<int32_t>(data[4]) << 4) |
        (static_cast<int32_t>(data[5]) >> 4);

    printf(
        "BMP280 RAW: T=%ld P=%ld\n",
        static_cast<long>(temperature),
        static_cast<long>(pressure));

    return true;
}

// =============================================================================
// Temperature compensation
//
// This is intentionally mathematically equivalent to the Python implementation:
//
// var1 = (adc_t / 16384 - T1 / 1024) * T2
//
// var2 = ((adc_t / 131072 - T1 / 8192)^2) * T3
//
// t_fine = var1 + var2
//
// temperature = t_fine / 5120
// =============================================================================

float BMP280::compensateTemperature(
    int32_t rawTemperature)
{
    float var1 =
        (static_cast<float>(rawTemperature) /
             16384.0f -
         static_cast<float>(dig_T1) /
             1024.0f) *
        static_cast<float>(dig_T2);

    float var2 =
        (static_cast<float>(rawTemperature) /
             131072.0f -
         static_cast<float>(dig_T1) /
             8192.0f);

    var2 =
        var2 *
        var2 *
        static_cast<float>(dig_T3);

    temperatureFine =
        var1 + var2;

    float temperatureC =
        temperatureFine /
        5120.0f;

    return temperatureC;
}

// =============================================================================
// Pressure compensation
//
// Equivalent to the Python floating-point implementation.
// =============================================================================

float BMP280::compensatePressure(
    int32_t rawPressure)
{
    float var1 =
        temperatureFine / 2.0f -
        64000.0f;

    float var2 =
        var1 *
        var1 *
        static_cast<float>(dig_P6) /
        32768.0f;

    var2 +=
        var1 *
        static_cast<float>(dig_P5) *
        2.0f;

    var2 =
        var2 / 4.0f +
        static_cast<float>(dig_P4) *
            65536.0f;

    var1 =
        (static_cast<float>(dig_P3) *
             var1 *
             var1 /
             524288.0f +
         static_cast<float>(dig_P2) *
             var1) /
        524288.0f;

    var1 =
        (1.0f +
         var1 / 32768.0f) *
        static_cast<float>(dig_P1);

    if (var1 == 0.0f)
    {
        return 0.0f;
    }

    float p =
        1048576.0f -
        static_cast<float>(rawPressure);

    p =
        (p -
         var2 / 4096.0f) *
        6250.0f /
        var1;

    p +=
        (static_cast<float>(dig_P9) *
             p *
             p /
             2147483648.0f +
         p *
             static_cast<float>(dig_P8) /
             32768.0f +
         static_cast<float>(dig_P7)) /
        16.0f;

    return p;
}

// =============================================================================
// Altitude
//
// Python:
//
// 44330.0 * (1.0 - (pressure / sea_level) ** 0.1903)
// =============================================================================

float BMP280::altitudeFromPressure(
    float pressureHpa)
{
    if (pressureHpa <= 0.0f ||
        seaLevelHpa <= 0.0f)
    {
        return 0.0f;
    }

    return 44330.0f *
           (1.0f -
            std::pow(
                pressureHpa /
                    seaLevelHpa,
                0.1903f));
}

// =============================================================================
// Test
//
// Equivalent to:
//
// start = time.ticks_ms()
//
// while timeout:
//
//     if update():
//         samples += 1
//
//     if samples >= 3:
//         return True
//
//     sleep_ms(200)
// =============================================================================

bool BMP280::test(
    uint32_t timeoutMs)
{
    if (!initialized)
    {
        return false;
    }

    absolute_time_t start =
        get_absolute_time();

    uint32_t samples = 0;

    while (
        absolute_time_diff_us(
            start,
            get_absolute_time()) <
        static_cast<int64_t>(timeoutMs) * 1000)
    {
        Measurements measurements{};

        if (read(measurements))
        {
            samples++;

            if (samples >= 3)
            {
                return true;
            }
        }

        /*
         * Same sampling interval as Python.
         */

        sleep_ms(200);
    }

    return false;
}

// =============================================================================
// Read register
// =============================================================================

bool BMP280::readRegister(
    uint8_t reg,
    uint8_t &value)
{
    return readRegisters(
        reg,
        &value,
        1);
}

// =============================================================================
// Write register
// =============================================================================

bool BMP280::writeRegister(
    uint8_t reg,
    uint8_t value)
{
    uint8_t data[2] =
        {
            reg,
            value};

    int result =
        bus.write(
            address,
            data,
            2,
            false);

    return result == 2;
}

// =============================================================================
// Read registers
// =============================================================================

bool BMP280::readRegisters(
    uint8_t reg,
    uint8_t *data,
    uint32_t length)
{
    if (data == nullptr ||
        length == 0)
    {
        return false;
    }

    int result =
        bus.writeRead(
            address,
            &reg,
            1,
            data,
            length);

    return result ==
           static_cast<int>(length);
}