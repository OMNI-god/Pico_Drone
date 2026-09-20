#include <cstdio>
#include <cstdint>

#include "pico/stdlib.h"
#include "pico/time.h"

#include "FreeRTOS.h"
#include "task.h"

// ============================================================
// Drivers
// ============================================================

#include "Elrs.h"
#include "I2c.h"
#include "ICM20948.h"

// ============================================================
// RC
// ============================================================

#include "CrsfParser.h"
#include "RCInput.h"
#include "RCPublisher.h"
#include "RCTask.h"

// ============================================================
// Flight / Sensor
// ============================================================

#include "IMUAdapter.h"
#include "IMUCalibration.h"
#include "IMUPublisher.h"
#include "IMUTask.h"

// ============================================================
// I2C configuration
// ============================================================

#define IMU_I2C i2c1

constexpr uint8_t I2C_SDA_PIN = 2;
constexpr uint8_t I2C_SCL_PIN = 3;

constexpr uint32_t I2C_BAUDRATE = 400000;

// ============================================================
// CRSF configuration
// ============================================================

#define CRSF_UART uart1

constexpr uint32_t CRSF_BAUDRATE = 420000;

constexpr uint8_t UART_TX_PIN = 4;
constexpr uint8_t UART_RX_PIN = 5;

// ============================================================
// Hardware objects
// ============================================================

static I2c i2c(
    IMU_I2C,
    I2C_BAUDRATE,
    I2C_SDA_PIN,
    I2C_SCL_PIN);

static ICM20948 icm20948(i2c);

// ============================================================
// IMU processing
// ============================================================

static IMUAdapter imuAdapter(icm20948);

static IMUCalibration imuCalibration;

static IMUPublisher imuPublisher;

static IMUTask imuTask(
    imuAdapter,
    imuCalibration,
    imuPublisher);

// ============================================================
// RC processing
// ============================================================

static Elrs elrs(
    CRSF_UART,
    CRSF_BAUDRATE,
    UART_RX_PIN,
    UART_TX_PIN);

static CrsfParser crsfParser;

static RCInput rcInput;

static RCPublisher rcPublisher;

static RCTask rcTask(
    elrs,
    crsfParser,
    rcInput,
    rcPublisher);

// ============================================================
// Diagnostics task
// ============================================================

static void diagnosticsTask(void *parameter)
{
    (void)parameter;

    uint32_t previousSequence = 0;

    while (true)
    {
        // ====================================================
        // IMU snapshot
        // ====================================================

        IMUSnapshot imuSnapshot{};

        const bool imuValid =
            imuPublisher.read(imuSnapshot);

        const uint32_t currentSequence =
            imuSnapshot.sequence;

        const uint32_t samplesPerSecond =
            currentSequence -
            previousSequence;

        previousSequence =
            currentSequence;

        // ====================================================
        // RC state
        // ====================================================

        RCState rcState{};

        const bool rcStateValid =
            rcPublisher.read(rcState);

        const RCHealth &rcHealth =
            rcTask.getHealth();

        // ====================================================
        // Header
        // ====================================================

        printf("\n");
        printf("============================================================\n");
        printf("                  FLIGHT CONTROLLER STATUS\n");
        printf("============================================================\n");

        // ====================================================
        // IMU
        // ====================================================

        printf("IMU\n");

        printf(
            "  Valid       : %s\n",
            imuValid ? "YES" : "NO");

        printf(
            "  Calibrated  : %s\n",
            imuSnapshot.calibrated
                ? "YES"
                : "NO");

        printf(
            "  Sequence    : %lu\n",
            static_cast<unsigned long>(
                currentSequence));

        printf(
            "  Rate        : %lu samples/sec\n",
            static_cast<unsigned long>(
                samplesPerSecond));

        if (imuValid)
        {
            printf(
                "  Accel       : "
                "X=%8.4f "
                "Y=%8.4f "
                "Z=%8.4f g\n",

                imuSnapshot.state.accelX,
                imuSnapshot.state.accelY,
                imuSnapshot.state.accelZ);

            printf(
                "  Gyro        : "
                "X=%8.3f "
                "Y=%8.3f "
                "Z=%8.3f deg/s\n",

                imuSnapshot.state.gyroX,
                imuSnapshot.state.gyroY,
                imuSnapshot.state.gyroZ);

            printf(
                "  Timestamp   : %llu us\n",

                static_cast<unsigned long long>(
                    imuSnapshot.state.timestampUs));
        }

        // ====================================================
        // Calibration
        // ====================================================

        printf(
            "  Calibration : %u / %u\n",

            static_cast<unsigned>(
                imuCalibration.getSampleCount()),

            static_cast<unsigned>(
                imuCalibration.getRequiredSampleCount()));

        // ====================================================
        // RC
        // ====================================================

        printf("\n");
        printf("RC\n");

        printf(
            "  Frames      : %lu\n",
            static_cast<unsigned long>(
                rcHealth.totalFrames));

        printf(
            "  Rate        : %lu frames/sec\n",
            static_cast<unsigned long>(
                rcHealth.framesLastSecond));

        printf(
            "  Connected   : %s\n",
            rcHealth.connected
                ? "YES"
                : "NO");

        printf(
            "  Healthy     : %s\n",
            rcHealth.healthy
                ? "YES"
                : "NO");

        printf(
            "  Failsafe    : %s\n",
            rcHealth.failsafe
                ? "YES"
                : "NO");

        // ----------------------------------------------------
        // Frame age
        // ----------------------------------------------------

        if (rcHealth.lastFrameTimeUs == 0)
        {
            printf(
                "  Frame age   : NEVER\n");
        }
        else
        {
            printf(
                "  Frame age   : %llu us\n",

                static_cast<unsigned long long>(
                    rcHealth.frameAgeUs));
        }

        // ----------------------------------------------------
        // Received bytes
        // ----------------------------------------------------

        printf(
            "  Bytes       : %lu\n",

            static_cast<unsigned long>(
                rcHealth.totalBytes));

        // ----------------------------------------------------
        // Published RC state
        // ----------------------------------------------------

        printf(
            "  State valid : %s\n",
            rcStateValid
                ? "YES"
                : "NO");

        // ====================================================
        // Raw RC channels
        // ====================================================

        printf(
            "  Channels    : "
            "%u %u %u %u | "
            "%u %u %u %u\n",

            rcState.rawChannels[0],
            rcState.rawChannels[1],
            rcState.rawChannels[2],
            rcState.rawChannels[3],

            rcState.rawChannels[4],
            rcState.rawChannels[5],
            rcState.rawChannels[6],
            rcState.rawChannels[7]);

        printf(
            "                "
            "%u %u %u %u | "
            "%u %u %u %u\n",

            rcState.rawChannels[8],
            rcState.rawChannels[9],
            rcState.rawChannels[10],
            rcState.rawChannels[11],

            rcState.rawChannels[12],
            rcState.rawChannels[13],
            rcState.rawChannels[14],
            rcState.rawChannels[15]);

        // ====================================================
        // Processed RC state
        // ====================================================

        printf(
            "  Roll        : %7.3f\n",
            rcState.roll);

        printf(
            "  Pitch       : %7.3f\n",
            rcState.pitch);

        printf(
            "  Yaw         : %7.3f\n",
            rcState.yaw);

        printf(
            "  Throttle    : %7.3f\n",
            rcState.throttle);

        printf(
            "  Arm         : %s\n",
            rcState.arm
                ? "ON"
                : "OFF");

        printf(
            "  Beeper      : %s\n",
            rcState.beeper
                ? "ON"
                : "OFF");

        printf(
            "  Calibration : %s\n",
            rcState.calibration
                ? "ON"
                : "OFF");

        // ====================================================
        // Flight mode
        // ====================================================

        printf(
            "  Flight mode : ");

        switch (rcState.flightMode)
        {
        case RCFlightMode::Angle:
            printf("ANGLE\n");
            break;

        case RCFlightMode::Horizon:
            printf("HORIZON\n");
            break;

        case RCFlightMode::Acro:
            printf("ACRO\n");
            break;

        default:
            printf("UNKNOWN\n");
            break;
        }

        // ====================================================
        // RC timestamp / frame counter
        // ====================================================

        printf(
            "  Frame count : %lu\n",

            static_cast<unsigned long>(
                rcState.frameCounter));

        printf(
            "  Timestamp   : %llu us\n",

            static_cast<unsigned long long>(
                rcState.lastUpdateUs));

        // ====================================================
        // FreeRTOS / system
        // ====================================================

        printf("\n");
        printf("SYSTEM\n");

        printf(
            "  Heap free   : %lu bytes\n",

            static_cast<unsigned long>(
                xPortGetFreeHeapSize()));

        printf("============================================================\n");

        // ====================================================
        // Diagnostics period
        // ====================================================

        vTaskDelay(
            pdMS_TO_TICKS(1000));
    }
}

// ============================================================
// Main
// ============================================================

int main()
{
    // ========================================================
    // Standard I/O
    // ========================================================

    stdio_init_all();

    // Give USB CDC time to initialize.
    sleep_ms(2000);

    printf("\n");
    printf("============================================================\n");
    printf("              RASPBERRY PI PICO 2 FC\n");
    printf("============================================================\n");

    printf(
        "FreeRTOS configuration loaded.\n");

    // ========================================================
    // I2C
    // ========================================================

    printf("\n");
    printf("Initializing I2C...\n");

    if (!i2c.initialize())
    {
        printf(
            "ERROR: I2C initialization failed.\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf(
        "I2C initialized.\n");

    printf(
        "  SDA       : GPIO %u\n",
        static_cast<unsigned>(
            I2C_SDA_PIN));

    printf(
        "  SCL       : GPIO %u\n",
        static_cast<unsigned>(
            I2C_SCL_PIN));

    printf(
        "  Baudrate  : %lu Hz\n",
        static_cast<unsigned long>(
            I2C_BAUDRATE));

    // ========================================================
    // ICM20948
    // ========================================================

    printf("\n");
    printf("Initializing ICM20948...\n");

    if (!icm20948.initialize())
    {
        printf(
            "ERROR: ICM20948 initialization failed.\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf(
        "ICM20948 initialized.\n");

    printf(
        "Address: 0x%02X\n",
        icm20948.getAddress());

    // ========================================================
    // ELRS
    // ========================================================

    printf("\n");
    printf("Initializing ELRS...\n");

    if (!elrs.initialize())
    {
        printf(
            "ERROR: ELRS initialization failed.\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf(
        "ELRS initialized.\n");

    printf(
        "  UART      : UART1\n");

    printf(
        "  RX GPIO   : %u\n",
        static_cast<unsigned>(
            UART_RX_PIN));

    printf(
        "  TX GPIO   : %u\n",
        static_cast<unsigned>(
            UART_TX_PIN));

    printf(
        "  Baudrate  : %lu\n",
        static_cast<unsigned long>(
            CRSF_BAUDRATE));

    // ========================================================
    // RC task
    // ========================================================

    printf("\n");
    printf("Starting RC task...\n");

    if (!rcTask.start(
            "RC",
            1024,
            6))
    {
        printf(
            "ERROR: RC task creation failed.\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf(
        "RC task started.\n");

    // ========================================================
    // IMU task
    // ========================================================

    printf(
        "Starting IMU task...\n");

    if (!imuTask.start())
    {
        printf(
            "ERROR: IMU task creation failed.\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf(
        "IMU task started.\n");

    // ========================================================
    // Diagnostics task
    // ========================================================

    printf(
        "Creating diagnostics task...\n");

    if (xTaskCreate(
            diagnosticsTask,
            "Diagnostics",
            2048,
            nullptr,
            2,
            nullptr) != pdPASS)
    {
        printf(
            "ERROR: Diagnostics task creation failed.\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf(
        "Diagnostics task created.\n");

    // ========================================================
    // Scheduler
    // ========================================================

    printf("\n");
    printf("============================================================\n");
    printf("Starting FreeRTOS scheduler...\n");
    printf("============================================================\n");

    vTaskStartScheduler();

    // ========================================================
    // Scheduler should never return
    // ========================================================

    printf(
        "ERROR: FreeRTOS scheduler returned!\n");

    while (true)
    {
        tight_loop_contents();
    }

    return 0;
}