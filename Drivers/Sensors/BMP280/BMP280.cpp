#include "BMP280.h"

#include "pico/stdlib.h"

#include <cstdio>

// =============================================================================
// Constructor
// =============================================================================

BMP280::BMP280(
    II2c &bus,
    uint8_t address)
    : bus(bus),
      address(address)
{
}

// =============================================================================
// Initialize
// =============================================================================

bool BMP280::initialize()
{
    initialized = false;

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

    if (id != DEVICE_ID)
    {
        printf(
            "BMP280: invalid CHIP_ID, "
            "expected 0x58\n");

        return false;
    }

    // =========================================================================
    // SOFTWARE RESET
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

    // Datasheet reset startup time.
    sleep_ms(5);

    // =========================================================================
    // Wait for NVM calibration update
    // =========================================================================

    bool nvmReady = false;

    for (int i = 0; i < 100; ++i)
    {
        uint8_t status = 0;

        if (!readRegister(
                REG_STATUS,
                status))
        {
            printf(
                "BMP280: STATUS read failed\n");

            return false;
        }

        printf(
            "BMP280 INIT STATUS[%d] = 0x%02X\n",
            i,
            status);

        if ((status & STATUS_IM_UPDATE) == 0)
        {
            nvmReady = true;
            break;
        }

        sleep_ms(1);
    }

    if (!nvmReady)
    {
        printf(
            "BMP280: NVM update timeout\n");

        return false;
    }

    // =========================================================================
    // Read calibration
    // =========================================================================

    if (!readCalibration())
    {
        printf(
            "BMP280: calibration read failed\n");

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
    // Start in sleep mode
    // =========================================================================

    printf(
        "BMP280: writing CTRL_MEAS sleep = 0x%02X\n",
        CTRL_MEAS_SLEEP);

    if (!writeRegister(
            REG_CTRL_MEAS,
            CTRL_MEAS_SLEEP))
    {
        printf(
            "BMP280: CTRL_MEAS write failed\n");

        return false;
    }

    sleep_ms(2);

    // =========================================================================
    // Read back configuration
    // =========================================================================

    uint8_t config = 0;
    uint8_t ctrl = 0;

    if (!readRegister(
            REG_CONFIG,
            config))
    {
        printf(
            "BMP280: CONFIG readback failed\n");

        return false;
    }

    if (!readRegister(
            REG_CTRL_MEAS,
            ctrl))
    {
        printf(
            "BMP280: CTRL_MEAS readback failed\n");

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
            "BMP280: WARNING CONFIG mismatch\n");
    }

    if (ctrl != CTRL_MEAS_SLEEP)
    {
        printf(
            "BMP280: WARNING CTRL_MEAS mismatch\n");
    }

    initialized = true;

    printf(
        "BMP280: initialization successful\n");

    return true;
}

// =============================================================================
// Trigger forced measurement
// =============================================================================

bool BMP280::triggerMeasurement()
{
    printf(
        "BMP280: triggering forced measurement\n");

    if (!writeRegister(
            REG_CTRL_MEAS,
            CTRL_MEAS_FORCED))
    {
        printf(
            "BMP280: failed to trigger measurement\n");

        return false;
    }

    uint8_t ctrl = 0;

    if (!readRegister(
            REG_CTRL_MEAS,
            ctrl))
    {
        printf(
            "BMP280: CTRL_MEAS read failed\n");

        return false;
    }

    printf(
        "BMP280: CTRL_MEAS after trigger = 0x%02X\n",
        ctrl);

    // =========================================================================
    // Important:
    //
    // The mode bits are:
    //
    // 00 = sleep
    // 01 = forced
    // 10/11 = normal
    //
    // Reading 0x25 means:
    //
    // 010 010 01
    //       ^^^
    //       forced mode
    //
    // It is valid immediately after triggering.
    // =========================================================================

    return true;
}

// =============================================================================
// Wait for measurement
// =============================================================================

bool BMP280::waitForMeasurement()
{
    // =========================================================================
    // IMPORTANT:
    //
    // Do not immediately assume STATUS=0 means conversion is complete.
    //
    // Forced measurement takes time.
    //
    // With:
    //
    // Temperature x2
    // Pressure    x16
    //
    // conversion is approximately tens of milliseconds.
    //
    // Give the sensor some time before polling.
    // =========================================================================

    sleep_ms(5);

    for (int i = 0; i < 100; ++i)
    {
        uint8_t status = 0;

        if (!readRegister(
                REG_STATUS,
                status))
        {
            printf(
                "BMP280: STATUS read failed\n");

            return false;
        }

        printf(
            "BMP280 STATUS[%d] = 0x%02X\n",
            i,
            status);

        // ---------------------------------------------------------------------
        // NVM update
        // ---------------------------------------------------------------------

        if (status & STATUS_IM_UPDATE)
        {
            sleep_ms(1);
            continue;
        }

        // ---------------------------------------------------------------------
        // Measurement still running
        // ---------------------------------------------------------------------

        if (status & STATUS_MEASURING)
        {
            sleep_ms(2);
            continue;
        }

        // ---------------------------------------------------------------------
        // Measurement complete
        // ---------------------------------------------------------------------

        printf(
            "BMP280: MEASUREMENT COMPLETE\n");

        return true;
    }

    printf(
        "BMP280: measurement timeout\n");

    return false;
}

// =============================================================================
// Read
// =============================================================================

bool BMP280::read(
    Measurements &measurements)
{
    if (!initialized)
    {
        printf(
            "BMP280: not initialized\n");

        return false;
    }

    // =========================================================================
    // Trigger
    // =========================================================================

    if (!triggerMeasurement())
    {
        return false;
    }

    // =========================================================================
    // Wait
    // =========================================================================

    if (!waitForMeasurement())
    {
        return false;
    }

    // =========================================================================
    // Read raw values
    // =========================================================================

    int32_t rawTemperature = 0;
    int32_t rawPressure = 0;

    if (!readRaw(
            rawTemperature,
            rawPressure))
    {
        return false;
    }

    // =========================================================================
    // Validate raw values
    // =========================================================================

    if (rawTemperature <= 0 ||
        rawTemperature > 0xFFFFF)
    {
        printf(
            "BMP280: invalid raw temperature = %ld\n",
            static_cast<long>(rawTemperature));

        return false;
    }

    if (rawPressure <= 0 ||
        rawPressure > 0xFFFFF)
    {
        printf(
            "BMP280: invalid raw pressure = %ld\n",
            static_cast<long>(rawPressure));

        return false;
    }

    // =========================================================================
    // Temperature compensation
    // =========================================================================

    measurements.temperatureC =
        compensateTemperature(
            rawTemperature);

    // =========================================================================
    // Pressure compensation
    // =========================================================================

    measurements.pressurePa =
        compensatePressure(
            rawPressure);

    if (measurements.pressurePa <= 0.0f)
    {
        printf(
            "BMP280: invalid compensated pressure = %.2f Pa\n",
            measurements.pressurePa);

        return false;
    }

    measurements.pressureHpa =
        measurements.pressurePa / 100.0f;

    printf(
        "BMP280: T=%.2f C "
        "P=%.2f Pa "
        "%.2f hPa\n",
        measurements.temperatureC,
        measurements.pressurePa,
        measurements.pressureHpa);

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
// Check connection
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

    return id == DEVICE_ID;
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
    // Print
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

    printf("------------------\n");
    printf("\n");

    // =========================================================================
    // Basic validation
    // =========================================================================

    if (dig_T1 == 0)
    {
        printf(
            "BMP280: invalid T1\n");

        return false;
    }

    if (dig_P1 == 0)
    {
        printf(
            "BMP280: invalid P1\n");

        return false;
    }

    return true;
}

// =============================================================================
// Read raw measurement
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
            "BMP280: measurement data read failed\n");

        return false;
    }

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
// =============================================================================

float BMP280::compensateTemperature(
    int32_t rawTemperature)
{
    int32_t var1;
    int32_t var2;

    var1 =
        ((((rawTemperature >> 3) -
           (static_cast<int32_t>(dig_T1) << 1))) *
         static_cast<int32_t>(dig_T2)) >>
        11;

    var2 =
        (((((rawTemperature >> 4) -
            static_cast<int32_t>(dig_T1)) *
           ((rawTemperature >> 4) -
            static_cast<int32_t>(dig_T1))) >>
          12) *
         static_cast<int32_t>(dig_T3)) >>
        14;

    temperatureFine =
        var1 + var2;

    int32_t temperature =
        (temperatureFine * 5 + 128) >> 8;

    return temperature / 100.0f;
}

// =============================================================================
// Pressure compensation
// =============================================================================

float BMP280::compensatePressure(
    int32_t rawPressure)
{
    int64_t var1;
    int64_t var2;

    var1 =
        static_cast<int64_t>(temperatureFine) -
        128000;

    var2 =
        var1 *
        var1 *
        static_cast<int64_t>(dig_P6);

    var2 +=
        (var1 *
         static_cast<int64_t>(dig_P5))
        << 17;

    var2 +=
        static_cast<int64_t>(dig_P4)
        << 35;

    var1 =
        ((var1 *
          var1 *
          static_cast<int64_t>(dig_P3)) >>
         8) +
        ((var1 *
          static_cast<int64_t>(dig_P2))
         << 12);

    var1 =
        (((static_cast<int64_t>(1) << 47) +
          var1) *
         static_cast<int64_t>(dig_P1)) >>
        33;

    if (var1 == 0)
    {
        return 0.0f;
    }

    int64_t p =
        1048576 -
        static_cast<int64_t>(rawPressure);

    p =
        (((p << 31) - var2) * 3125) /
        var1;

    var1 =
        (static_cast<int64_t>(dig_P9) *
         (p >> 13) *
         (p >> 13)) >>
        25;

    var2 =
        (static_cast<int64_t>(dig_P8) *
         p) >>
        19;

    p =
        ((p + var1 + var2) >> 8) +
        (static_cast<int64_t>(dig_P7) << 4);

    return p / 256.0f;
}

// =============================================================================
// Read single register
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
// Write single register
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
            sizeof(data),
            false);

    if (result !=
        static_cast<int>(sizeof(data)))
    {
        printf(
            "BMP280: register write failed "
            "REG=0x%02X VALUE=0x%02X RESULT=%d\n",
            reg,
            value,
            result);

        return false;
    }

    return true;
}

// =============================================================================
// Read multiple registers
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

    if (result !=
        static_cast<int>(length))
    {
        printf(
            "BMP280: register read failed "
            "REG=0x%02X LENGTH=%lu RESULT=%d\n",
            reg,
            static_cast<unsigned long>(length),
            result);

        return false;
    }

    return true;
}