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
#include "GY271.h"
#include "BMP280.h"
#include "ADXL345.h"
#include "Gps.h"
#include "Esc.h"

// ============================================================
// Managers
// ============================================================

#include "SensorManager.h"

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
#include "AttitudeEstimator.h"

// ============================================================
// Flight / Safety
// ============================================================

#include "SafetyManager.h"
#include "SafetyPublisher.h"
#include "SafetyTask.h"

// ============================================================
// Flight / Flight control
// ============================================================

#include "FlightController.h"
#include "FlightControlTask.h"

// ============================================================
// Flight / Telemetry
// ============================================================

#include "CrsfTelemetry.h"
#include "TelemetryTask.h"

// ============================================================
// I2C configuration
// ============================================================

#define IMU_I2C i2c1

constexpr uint8_t I2C_SDA_PIN = 2;
constexpr uint8_t I2C_SCL_PIN = 3;

constexpr uint32_t I2C_BAUDRATE = 400000;

// ============================================================
// GPS configuration
//
// GP1513 / MC-1513 (MediaTek MT3339)
//
//     NMEA 0183
//     9600 baud, 8 data bits, no parity, 1 stop bit
//
// UART0 is used because UART1 carries the ELRS
// Crossfire link.
// ============================================================

#define GPS_UART uart0

constexpr uint32_t GPS_BAUDRATE = 9600;

constexpr uint8_t GPS_RX_PIN = 0;
constexpr uint8_t GPS_TX_PIN = 1;

// ============================================================
// CRSF configuration
// ============================================================

#define CRSF_UART uart1

constexpr uint32_t CRSF_BAUDRATE = 420000;

constexpr uint8_t UART_TX_PIN = 4;
constexpr uint8_t UART_RX_PIN = 5;

// ============================================================
// Motor configuration
//
// Motor order:
//
//     0 = front right   (GPIO 6)
//     1 = front left    (GPIO 7)
//     2 = rear right    (GPIO 8)
//     3 = rear left     (GPIO 9)
// ============================================================

constexpr uint8_t MOTOR_COUNT = 4;

constexpr uint8_t MOTOR_PINS[MOTOR_COUNT] =
    {
        6,
        7,
        8,
        9};

// ============================================================
// Hardware objects
// ============================================================

static I2c i2c(
    IMU_I2C,
    I2C_BAUDRATE,
    I2C_SDA_PIN,
    I2C_SCL_PIN);

static ICM20948 icm20948(i2c);

static GY271 gy271(i2c);

static BMP280 bmp280(i2c);

static ADXL345 adxl345(i2c);

static Gps gps(
    GPS_UART,
    GPS_BAUDRATE,
    GPS_RX_PIN,
    GPS_TX_PIN);

static SensorManager sensorManager(
    icm20948,
    gy271,
    bmp280,
    adxl345,
    gps);

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

static AttitudeEstimator attitudeEstimator;

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
// Safety
// ============================================================

static SafetyManager safetyManager;

static SafetyPublisher safetyPublisher;

static SafetyTask safetyTask(
    rcPublisher,
    rcTask,
    imuPublisher,
    safetyManager,
    safetyPublisher);

// ============================================================
// Flight control
// ============================================================

static FlightController flightController;

static Esc motor0(MOTOR_PINS[0]);
static Esc motor1(MOTOR_PINS[1]);
static Esc motor2(MOTOR_PINS[2]);
static Esc motor3(MOTOR_PINS[3]);

static Esc *MOTORS[MOTOR_COUNT] =
    {
        &motor0,
        &motor1,
        &motor2,
        &motor3};

static FlightControlTask flightControlTask(
    rcPublisher,
    imuPublisher,
    safetyPublisher,
    flightController,
    attitudeEstimator,
    MOTORS,
    MOTOR_COUNT);

// ============================================================
// Telemetry
// ============================================================

static CrsfTelemetry crsfTelemetry;

static TelemetryTask telemetryTask(
    rcPublisher,
    imuPublisher,
    sensorManager,
    flightController,
    crsfTelemetry,
    elrs);

// ============================================================
// Diagnostics task
// ============================================================

static void diagnosticsTask(void *parameter)
{
    (void)parameter;

    uint32_t previousSequence = 0;

    bool previousCalibrationSwitch = false;

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

        /*
         * The calibration switch resets the IMU gyro
         * calibration. The request is forwarded to the
         * IMU task, which owns the calibration object.
         */

        if (rcState.calibration &&
            !previousCalibrationSwitch)
        {
            imuTask.requestCalibrationReset();
        }

        previousCalibrationSwitch =
            rcState.calibration;

        // ====================================================
        // Safety state
        // ====================================================

        SafetySnapshot safetySnapshot{};

        const bool safetyValid =
            safetyPublisher.read(safetySnapshot);

        // ====================================================
        // Flight controller state
        // ====================================================

        FlightControllerState flightState{};

        flightControlTask.readState(flightState);

        // ====================================================
        // Telemetry state
        // ====================================================

        TelemetryState telemetryState{};

        telemetryTask.readState(telemetryState);

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
        // Attitude
        // ====================================================

        printf(
            "  Attitude    : "
            "Roll=%7.2f "
            "Pitch=%7.2f "
            "Yaw=%7.2f deg\n",

            flightState.attitudeRoll,
            flightState.attitudePitch,
            flightState.attitudeYaw);

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
        // Safety
        // ====================================================

        printf("\n");
        printf("SAFETY\n");

        printf(
            "  Valid       : %s\n",
            safetyValid ? "YES" : "NO");

        printf(
            "  State       : ");

        switch (safetySnapshot.state)
        {
        case SafetyState::Disarmed:
            printf("DISARMED\n");
            break;

        case SafetyState::Arming:
            printf("ARMING\n");
            break;

        case SafetyState::Armed:
            printf("ARMED\n");
            break;

        case SafetyState::Failsafe:
            printf("FAILSAFE\n");
            break;

        default:
            printf("UNKNOWN\n");
            break;
        }

        printf(
            "  RC          : valid=%s failsafe=%s healthy=%s\n",

            safetySnapshot.rcValid ? "YES" : "NO",
            safetySnapshot.rcFailsafe ? "YES" : "NO",
            safetySnapshot.rcHealthy ? "YES" : "NO");

        printf(
            "  Throttle low: %s\n",

            safetySnapshot.throttleLow ? "YES" : "NO");

        printf(
            "  Arm request : %s\n",

            safetySnapshot.armRequested ? "YES" : "NO");

        printf(
            "  IMU         : valid=%s calibrated=%s fresh=%s\n",

            safetySnapshot.imuValid ? "YES" : "NO",
            safetySnapshot.imuCalibrated ? "YES" : "NO",
            safetySnapshot.imuDataFresh ? "YES" : "NO");

        // ----------------------------------------------------
        // Faults
        // ----------------------------------------------------

        printf(
            "  Faults      : ");

        if (safetySnapshot.faults == SafetyFault::None)
        {
            printf("NONE\n");
        }
        else
        {
            if (hasFault(
                    safetySnapshot.faults,
                    SafetyFault::RcInvalid))
            {
                printf("RC_INVALID ");
            }

            if (hasFault(
                    safetySnapshot.faults,
                    SafetyFault::RcFailsafe))
            {
                printf("RC_FAILSAFE ");
            }

            if (hasFault(
                    safetySnapshot.faults,
                    SafetyFault::RcUnhealthy))
            {
                printf("RC_UNHEALTHY ");
            }

            if (hasFault(
                    safetySnapshot.faults,
                    SafetyFault::ThrottleNotLow))
            {
                printf("THROTTLE_NOT_LOW ");
            }

            if (hasFault(
                    safetySnapshot.faults,
                    SafetyFault::ImuInvalid))
            {
                printf("IMU_INVALID ");
            }

            if (hasFault(
                    safetySnapshot.faults,
                    SafetyFault::ImuNotCalibrated))
            {
                printf("IMU_NOT_CALIBRATED ");
            }

            if (hasFault(
                    safetySnapshot.faults,
                    SafetyFault::ImuDataStale))
            {
                printf("IMU_STALE ");
            }

            if (hasFault(
                    safetySnapshot.faults,
                    SafetyFault::ArmSwitchNotActive))
            {
                printf("ARM_SWITCH_OFF ");
            }

            printf("\n");
        }

        // ====================================================
        // GPS
        // ====================================================

        printf("\n");
        printf("GPS\n");

        printf(
            "  Valid       : %s\n",
            telemetryState.satellites > 0 ||
                    telemetryState.latitude != 0.0
                ? "YES"
                : "NO");

        printf(
            "  Satellites  : %u\n",
            static_cast<unsigned>(
                telemetryState.satellites));

        printf(
            "  Latitude    : %.7f\n",
            telemetryState.latitude);

        printf(
            "  Longitude   : %.7f\n",
            telemetryState.longitude);

        printf(
            "  Altitude    : %.2f m\n",
            telemetryState.gpsAltitude);

        printf(
            "  Speed       : %.2f m/s\n",
            telemetryState.groundSpeed);

        printf(
            "  Heading     : %.1f deg\n",
            telemetryState.gpsHeading);

        // ====================================================
        // Barometer
        // ====================================================

        BMP280::Measurements barometer{};

        const bool barometerValid =
            sensorManager.readBarometer(barometer);

        printf("\n");
        printf("BAROMETER\n");

        printf(
            "  Valid       : %s\n",
            barometerValid ? "YES" : "NO");

        if (barometerValid)
        {
            printf(
                "  Temperature : %.2f C\n",
                barometer.temperatureC);

            printf(
                "  Pressure    : %.2f hPa\n",
                barometer.pressureHpa);

            printf(
                "  Altitude    : %.2f m\n",
                barometer.altitudeM);
        }

        // ====================================================
        // Flight control
        // ====================================================

        printf("\n");
        printf("FLIGHT CONTROL\n");

        printf(
            "  Armed       : %s\n",
            flightState.armed ? "YES" : "NO");

        printf(
            "  Failsafe    : %s\n",
            flightState.failsafe ? "YES" : "NO");

        printf(
            "  Throttle    : %.3f\n",
            flightState.throttle);

        printf(
            "  Target rate : "
            "Roll=%7.1f "
            "Pitch=%7.1f "
            "Yaw=%7.1f deg/s\n",

            flightState.targetRollRate,
            flightState.targetPitchRate,
            flightState.targetYawRate);

        printf(
            "  Motors      : "
            "M0=%5.3f "
            "M1=%5.3f "
            "M2=%5.3f "
            "M3=%5.3f\n",

            flightState.motors[0],
            flightState.motors[1],
            flightState.motors[2],
            flightState.motors[3]);

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
    // Sensors
    // ========================================================

    if (!sensorManager.initialize())
    {
        printf(
            "ERROR: IMU is not available.\n");

        printf(
            "Check the I2C wiring and the sensor power.\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

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
    // Motors
    // ========================================================

    printf("\n");
    printf("Initializing motors...\n");

    for (uint8_t i = 0;
         i < MOTOR_COUNT;
         ++i)
    {
        MOTORS[i]->initialize();

        MOTORS[i]->setSpeed(1000);

        printf(
            "  Motor %u   : GPIO %u\n",
            static_cast<unsigned>(i),
            static_cast<unsigned>(
                MOTOR_PINS[i]));
    }

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
    // Safety task
    // ========================================================

    printf(
        "Starting safety task...\n");

    if (!safetyTask.start(
            "Safety",
            1024,
            7))
    {
        printf(
            "ERROR: Safety task creation failed.\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf(
        "Safety task started.\n");

    // ========================================================
    // Flight control task
    // ========================================================

    printf(
        "Starting flight control task...\n");

    if (!flightControlTask.start(
            "FlightControl",
            2048,
            6))
    {
        printf(
            "ERROR: Flight control task creation failed.\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf(
        "Flight control task started.\n");

    // ========================================================
    // Telemetry task
    // ========================================================

    printf(
        "Starting telemetry task...\n");

    if (!telemetryTask.start(
            "Telemetry",
            2048,
            4))
    {
        printf(
            "ERROR: Telemetry task creation failed.\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf(
        "Telemetry task started.\n");

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
