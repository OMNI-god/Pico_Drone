#include "BMP280.h"

BMP280::BMP280(
    II2c &bus,
    uint8_t address)
    : bus(bus),
      address(address)
{
}

// ====================================================================================
// Initialize
// ====================================================================================

bool BMP280::initialize()
{
    initialized = false;

    // -------------------------------------------------------------------------
    // Check device ID
    // -------------------------------------------------------------------------

    uint8_t id;

    if (!readRegister(
            REG_ID,
            id))
    {
        return false;
    }

    if (id != DEVICE_ID)
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Read factory calibration coefficients
    // -------------------------------------------------------------------------

    if (!readCalibration())
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Configure sensor
    // -------------------------------------------------------------------------

    /*
     * CONFIG register
     *
     * t_sb = 125 ms
     * filter = x4
     * spi3w_en = disabled
     *
     * 010 << 5
     * 010 << 2
     *
     * 0x48
     */

    const uint8_t config =
        (2 << 5) |
        (2 << 2);

    if (!writeRegister(
            REG_CONFIG,
            config))
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // CTRL_MEAS
    // -------------------------------------------------------------------------

    /*
     * Temperature oversampling = x2
     * Pressure oversampling    = x16
     * Mode                     = Normal
     *
     * osrs_t = 010
     * osrs_p = 101
     * mode   = 11
     */

    const uint8_t ctrlMeas =
        (2 << 5) |
        (5 << 2) |
        3;

    if (!writeRegister(
            REG_CTRL_MEAS,
            ctrlMeas))
    {
        return false;
    }

    initialized = true;

    return true;
}

// ====================================================================================
// Read Temperature
// ====================================================================================

bool BMP280::readTemperature(
    float &temperatureC)
{
    if (!initialized)
    {
        return false;
    }

    int32_t rawTemperature;
    int32_t rawPressure;

    if (!readRaw(
            rawTemperature,
            rawPressure))
    {
        return false;
    }

    temperatureC =
        compensateTemperature(
            rawTemperature);

    return true;
}

// ====================================================================================
// Read Pressure
// ====================================================================================

bool BMP280::readPressure(
    float &pressurePa)
{
    if (!initialized)
    {
        return false;
    }

    int32_t rawTemperature;
    int32_t rawPressure;

    if (!readRaw(
            rawTemperature,
            rawPressure))
    {
        return false;
    }

    // Pressure compensation requires
    // temperatureFine to be calculated first.

    compensateTemperature(
        rawTemperature);

    pressurePa =
        compensatePressure(
            rawPressure);

    return pressurePa > 0.0f;
}

// ====================================================================================
// Read Temperature + Pressure
// ====================================================================================

bool BMP280::read(
    Measurements &measurements)
{
    if (!initialized)
    {
        return false;
    }

    int32_t rawTemperature;
    int32_t rawPressure;

    if (!readRaw(
            rawTemperature,
            rawPressure))
    {
        return false;
    }

    measurements.temperatureC =
        compensateTemperature(
            rawTemperature);

    measurements.pressurePa =
        compensatePressure(
            rawPressure);

    if (measurements.pressurePa <= 0.0f)
    {
        return false;
    }

    measurements.pressureHpa =
        measurements.pressurePa / 100.0f;

    return true;
}

// ====================================================================================
// Check Connection
// ====================================================================================

bool BMP280::isConnected()
{
    uint8_t id;

    return readRegister(
               REG_ID,
               id) &&
           id == DEVICE_ID;
}

// ====================================================================================
// Read Calibration
// ====================================================================================

bool BMP280::readCalibration()
{
    uint8_t data[24];

    if (!readRegisters(
            REG_CALIB,
            data,
            sizeof(data)))
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Temperature calibration
    // -------------------------------------------------------------------------

    dig_T1 =
        static_cast<uint16_t>(
            data[0] |
            (static_cast<uint16_t>(data[1]) << 8));

    dig_T2 =
        static_cast<int16_t>(
            data[2] |
            (static_cast<uint16_t>(data[3]) << 8));

    dig_T3 =
        static_cast<int16_t>(
            data[4] |
            (static_cast<uint16_t>(data[5]) << 8));

    // -------------------------------------------------------------------------
    // Pressure calibration
    // -------------------------------------------------------------------------

    dig_P1 =
        static_cast<uint16_t>(
            data[6] |
            (static_cast<uint16_t>(data[7]) << 8));

    dig_P2 =
        static_cast<int16_t>(
            data[8] |
            (static_cast<uint16_t>(data[9]) << 8));

    dig_P3 =
        static_cast<int16_t>(
            data[10] |
            (static_cast<uint16_t>(data[11]) << 8));

    dig_P4 =
        static_cast<int16_t>(
            data[12] |
            (static_cast<uint16_t>(data[13]) << 8));

    dig_P5 =
        static_cast<int16_t>(
            data[14] |
            (static_cast<uint16_t>(data[15]) << 8));

    dig_P6 =
        static_cast<int16_t>(
            data[16] |
            (static_cast<uint16_t>(data[17]) << 8));

    dig_P7 =
        static_cast<int16_t>(
            data[18] |
            (static_cast<uint16_t>(data[19]) << 8));

    dig_P8 =
        static_cast<int16_t>(
            data[20] |
            (static_cast<uint16_t>(data[21]) << 8));

    dig_P9 =
        static_cast<int16_t>(
            data[22] |
            (static_cast<uint16_t>(data[23]) << 8));

    // dig_P1 must not be zero.
    return dig_P1 != 0;
}

// ====================================================================================
// Read Raw Sensor Data
// ====================================================================================

bool BMP280::readRaw(
    int32_t &temperature,
    int32_t &pressure)
{
    uint8_t data[6];

    if (!readRegisters(
            REG_DATA,
            data,
            sizeof(data)))
    {
        return false;
    }

    // -------------------------------------------------------------------------
    // Pressure
    // -------------------------------------------------------------------------

    pressure =
        (static_cast<int32_t>(data[0]) << 12) |
        (static_cast<int32_t>(data[1]) << 4) |
        (static_cast<int32_t>(data[2]) >> 4);

    // -------------------------------------------------------------------------
    // Temperature
    // -------------------------------------------------------------------------

    temperature =
        (static_cast<int32_t>(data[3]) << 12) |
        (static_cast<int32_t>(data[4]) << 4) |
        (static_cast<int32_t>(data[5]) >> 4);

    return true;
}

// ====================================================================================
// Temperature Compensation
// ====================================================================================

float BMP280::compensateTemperature(
    int32_t rawTemperature)
{
    float var1 =
        ((rawTemperature / 16384.0f) -
         (dig_T1 / 1024.0f)) *
        dig_T2;

    float var2 =
        ((rawTemperature / 131072.0f) -
         (dig_T1 / 8192.0f));

    var2 =
        var2 *
        var2 *
        dig_T3;

    temperatureFine =
        static_cast<int32_t>(
            var1 + var2);

    return (var1 + var2) /
           5120.0f;
}

// ====================================================================================
// Pressure Compensation
// ====================================================================================

float BMP280::compensatePressure(
    int32_t rawPressure)
{
    float var1 =
        temperatureFine / 2.0f -
        64000.0f;

    float var2 =
        var1 *
        var1 *
        dig_P6 /
        32768.0f;

    var2 +=
        var1 *
        dig_P5 *
        2.0f;

    var2 =
        var2 / 4.0f +
        dig_P4 *
            65536.0f;

    var1 =
        dig_P3 *
        var1 *
        var1 /
        524288.0f;

    var1 +=
        dig_P2 *
        var1 /
        524288.0f;

    var1 =
        (1.0f +
         var1 / 32768.0f) *
        dig_P1;

    if (var1 == 0.0f)
    {
        return 0.0f;
    }

    float pressure =
        1048576.0f -
        rawPressure;

    pressure =
        (pressure -
         var2 / 4096.0f) *
        6250.0f /
        var1;

    var1 =
        dig_P9 *
        pressure *
        pressure /
        2147483648.0f;

    var2 =
        pressure *
        dig_P8 /
        32768.0f;

    pressure +=
        (var1 +
         var2 +
         dig_P7) /
        16.0f;

    return pressure;
}

// ====================================================================================
// Read Register
// ====================================================================================

bool BMP280::readRegister(
    uint8_t reg,
    uint8_t &value)
{
    return readRegisters(
        reg,
        &value,
        1);
}

// ====================================================================================
// Write Register
// ====================================================================================

bool BMP280::writeRegister(
    uint8_t reg,
    uint8_t value)
{
    const uint8_t data[2] =
        {
            reg,
            value};

    return bus.write(
               address,
               data,
               sizeof(data)) == 2;
}

// ====================================================================================
// Read Registers
// ====================================================================================

bool BMP280::readRegisters(
    uint8_t reg,
    uint8_t *data,
    uint32_t length)
{
    if (data == nullptr || length == 0)
    {
        return false;
    }

    return bus.writeRead(
               address,
               &reg,
               1,
               data,
               length) ==
           static_cast<int>(length);
}