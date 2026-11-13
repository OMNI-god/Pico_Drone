#include "I2c.h"

#include "hardware/gpio.h"

#include "pico/time.h"

#include <cstdio>

namespace
{
/*
 * Whole-transaction transfer timeout.
 *
 * A blocking transfer without a timeout hangs forever when a
 * slave holds SDA low (for example after the MCU was reset in
 * the middle of a transaction). Five milliseconds is far more
 * than a 400 kHz transfer needs, and it bounds the failure.
 */
constexpr uint32_t kTransferTimeoutUs = 5000;

/*
 * Rate limit transfer error reports so a dead bus cannot
 * flood the console at the sensor sampling rate.
 */
bool shouldReportError()
{
    static uint64_t lastReportUs = 0;

    const uint64_t now = time_us_64();

    if (now - lastReportUs < 1000000ULL)
    {
        return false;
    }

    lastReportUs = now;

    return true;
}
} // namespace

// =============================================================================
// Constructor
// =============================================================================

I2c::I2c(
    i2c_inst_t *i2c,
    uint32_t baudrate,
    uint32_t sdaPin,
    uint32_t sclPin)
    : i2cInstance(i2c),
      baudrate(baudrate),
      sdaPin(sdaPin),
      sclPin(sclPin)
{
}

// =============================================================================
// Initialize
// =============================================================================

bool I2c::initialize()
{
    if (initialized)
    {
        return true;
    }

    if (i2cInstance == nullptr)
    {
        printf("I2C: invalid instance\n");
        return false;
    }

    /*
     * Recover the bus before handing it to the hardware
     * controller. The previous reset may have interrupted a
     * transfer and left a slave holding SDA low.
     */

    recoverBus();

    i2c_init(
        i2cInstance,
        baudrate);

    gpio_set_function(
        sdaPin,
        GPIO_FUNC_I2C);

    gpio_set_function(
        sclPin,
        GPIO_FUNC_I2C);

    gpio_pull_up(sdaPin);
    gpio_pull_up(sclPin);

    /*
     * Serialize access to the shared bus.
     *
     * Created before the scheduler starts, so the
     * default dynamic allocation is used.
     */

    if (busMutex == nullptr)
    {
        busMutex = xSemaphoreCreateMutex();
    }

    initialized = true;

    printf(
        "I2C initialized: SDA=%lu SCL=%lu Baudrate=%lu\n",
        static_cast<unsigned long>(sdaPin),
        static_cast<unsigned long>(sclPin),
        static_cast<unsigned long>(baudrate));

    return true;
}

// =============================================================================
// Write
// =============================================================================

int I2c::write(
    uint8_t address,
    const uint8_t *data,
    uint32_t length,
    bool nostop)
{
    if (!initialized)
    {
        printf("I2C: write called before initialize\n");
        return -1;
    }

    if (data == nullptr || length == 0)
    {
        printf("I2C: invalid write parameters\n");
        return -1;
    }

    acquireBus();

    int result =
        i2c_write_timeout_us(
            i2cInstance,
            address,
            data,
            length,
            nostop,
            kTransferTimeoutUs);

    releaseBus();

    if (result < 0 && shouldReportError())
    {
        printf(
            "I2C: write failed, address=0x%02X result=%d\n",
            address,
            result);
    }

    return result;
}

// =============================================================================
// Read
// =============================================================================

int I2c::read(
    uint8_t address,
    uint8_t *data,
    uint32_t length,
    bool nostop)
{
    if (!initialized)
    {
        printf("I2C: read called before initialize\n");
        return -1;
    }

    if (data == nullptr || length == 0)
    {
        printf("I2C: invalid read parameters\n");
        return -1;
    }

    acquireBus();

    int result =
        i2c_read_timeout_us(
            i2cInstance,
            address,
            data,
            length,
            nostop,
            kTransferTimeoutUs);

    releaseBus();

    if (result < 0 && shouldReportError())
    {
        printf(
            "I2C: read failed, address=0x%02X result=%d\n",
            address,
            result);
    }

    return result;
}

// =============================================================================
// Write + Read
//
// Performs:
//
// START
// ADDRESS + WRITE
// REGISTER
// REPEATED START
// ADDRESS + READ
// DATA
// STOP
//
// This is the transaction required by HMC5883L.
// =============================================================================

int I2c::writeRead(
    uint8_t address,
    const uint8_t *txData,
    uint32_t txLength,
    uint8_t *rxData,
    uint32_t rxLength)
{
    if (!initialized)
    {
        printf(
            "I2C: writeRead called before initialize\n");

        return -1;
    }

    if (txData == nullptr ||
        rxData == nullptr ||
        txLength == 0 ||
        rxLength == 0)
    {
        printf(
            "I2C: invalid writeRead parameters\n");

        return -1;
    }

    acquireBus();

    // -------------------------------------------------------------------------
    // Write register address.
    //
    // nostop = true
    //
    // This keeps the bus active and generates a repeated START before read.
    // -------------------------------------------------------------------------

    int writeResult =
        i2c_write_timeout_us(
            i2cInstance,
            address,
            txData,
            txLength,
            true,
            kTransferTimeoutUs);

    if (writeResult != static_cast<int>(txLength))
    {
        if (shouldReportError())
        {
            printf(
                "I2C: writeRead WRITE failed, address=0x%02X expected=%lu actual=%d\n",
                address,
                static_cast<unsigned long>(txLength),
                writeResult);
        }

        releaseBus();

        return -1;
    }

    // -------------------------------------------------------------------------
    // Read data.
    //
    // nostop = false
    //
    // This generates STOP after the read.
    // -------------------------------------------------------------------------

    int readResult =
        i2c_read_timeout_us(
            i2cInstance,
            address,
            rxData,
            rxLength,
            false,
            kTransferTimeoutUs);

    releaseBus();

    if (readResult != static_cast<int>(rxLength))
    {
        if (shouldReportError())
        {
            printf(
                "I2C: writeRead READ failed, address=0x%02X expected=%lu actual=%d\n",
                address,
                static_cast<unsigned long>(rxLength),
                readResult);
        }

        return -1;
    }

    return readResult;
}

// =============================================================================
// Bus ownership
// =============================================================================

void I2c::acquireBus()
{
    if (busMutex != nullptr)
    {
        xSemaphoreTake(
            busMutex,
            portMAX_DELAY);
    }
}

void I2c::releaseBus()
{
    if (busMutex != nullptr)
    {
        xSemaphoreGive(busMutex);
    }
}

// =============================================================================
// Bus recovery
// =============================================================================

void I2c::recoverBus()
{
    /*
     * Drive both lines as open-drain GPIOs so the bus can be
     * clocked manually. SDA stays an input unless a STOP has to
     * be generated.
     */

    gpio_init(sdaPin);
    gpio_set_dir(sdaPin, GPIO_IN);
    gpio_pull_up(sdaPin);

    gpio_init(sclPin);
    gpio_put(sclPin, 1);
    gpio_set_dir(sclPin, GPIO_OUT);
    gpio_pull_up(sclPin);

    sleep_us(5);

    /*
     * SDA high means the bus is idle. If a slave is still
     * holding it low, clock up to nine bits so it finishes the
     * byte it was sending and releases the line.
     */

    if (!gpio_get(sdaPin))
    {
        for (int i = 0;
             i < 9 && !gpio_get(sdaPin);
             ++i)
        {
            gpio_put(sclPin, 0);
            sleep_us(5);

            gpio_put(sclPin, 1);
            sleep_us(5);
        }

        /*
         * Generate a STOP: SDA low while SCL is high, then
         * release SDA.
         */

        gpio_put(sclPin, 0);
        sleep_us(5);

        gpio_put(sdaPin, 0);
        gpio_set_dir(sdaPin, GPIO_OUT);
        sleep_us(5);

        gpio_put(sclPin, 1);
        sleep_us(5);

        gpio_put(sdaPin, 1);
        sleep_us(5);

        gpio_set_dir(sdaPin, GPIO_IN);
    }
}
