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
#include "PioUartRx.h"
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
//     115200 baud, 8 data bits, no parity, 1 stop bit
//
// The GP1513 factory default is 9600 baud; this
// receiver is configured for 115200 baud. The PIO
// receiver drains its FIFO into a ring buffer from
// an interrupt, so no bytes are lost even though
// the telemetry task polls the GPS every 1 ms.
//
// The serial hardware is fixed:
//
//     WiFi  GPIO12/13  (UART0)
//     GPS   GPIO8/9    (UART1)
//     ELRS  GPIO4/5    (UART1)
//
// The RP2350 has only two UART peripherals, and both
// GPS and ELRS are wired to UART1 pins, so the GPS
// receive path is implemented in PIO on GPIO9 and the
// hardware UART1 is left to the ELRS/CRSF link.
//
// The GPS is receive-only: the module's RX input is
// held at the idle-high level on GPIO8 instead of
// being driven by a second UART TX.
//
// Wiring:
//
//     GPS TX  -> GPIO9 (PIO RX)
//     GPS RX  -> GPIO8 (held idle-high)
// ============================================================

constexpr uint32_t GPS_BAUDRATE = 115200;

constexpr uint8_t GPS_RX_PIN = 9;
constexpr uint8_t GPS_TX_PIN = 8;

// ============================================================
// CRSF configuration
//
// ELRS stays on UART1 GPIO4/5, which is where the
// receiver is physically wired.
//
// Wiring:
//
//     ELRS TX -> GPIO5 (Pico RX)
//     ELRS RX -> GPIO4 (Pico TX)
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
//     2 = rear right    (GPIO 10)
//     3 = rear left     (GPIO 11)
//
// Motors 2 and 3 must not use GPIO8/9: those pins
// belong to the GPS UART.
// ============================================================

constexpr uint8_t MOTOR_COUNT = 4;

constexpr uint8_t MOTOR_PINS[MOTOR_COUNT] =
    {
        6,
        7,
        10,
        11};

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

static PioUartRx gpsPioRx(GPS_RX_PIN);

static Gps gps(
    gpsPioRx,
    GPS_BAUDRATE,
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
// Diagnostics formatting helpers
// ============================================================

static const char *yesNo(bool value)
{
    return value ? "YES" : "NO";
}

static const char *okOrDash(bool value)
{
    return value ? "OK" : "--";
}

static const char *onOff(bool value)
{
    return value ? "ON" : "OFF";
}

static const char *flightModeName(RCFlightMode mode)
{
    switch (mode)
    {
    case RCFlightMode::Angle:
        return "ANGLE";

    case RCFlightMode::Horizon:
        return "HORIZON";

    case RCFlightMode::Acro:
        return "ACRO";

    default:
        return "UNKNOWN";
    }
}

static const char *safetyStateName(SafetyState state)
{
    switch (state)
    {
    case SafetyState::Disarmed:
        return "DISARMED";

    case SafetyState::Arming:
        return "ARMING";

    case SafetyState::Armed:
        return "ARMED";

    case SafetyState::Failsafe:
        return "FAILSAFE";

    default:
        return "UNKNOWN";
    }
}

static void printFaultList(SafetyFault faults)
{
    if (faults == SafetyFault::None)
    {
        printf("NONE");
        return;
    }

    bool first = true;

    const auto emit = [&first](const char *name)
    {
        printf("%s%s", first ? "" : " ", name);
        first = false;
    };

    if (hasFault(faults, SafetyFault::RcInvalid))
    {
        emit("RC_INVALID");
    }

    if (hasFault(faults, SafetyFault::RcFailsafe))
    {
        emit("RC_FAILSAFE");
    }

    if (hasFault(faults, SafetyFault::RcUnhealthy))
    {
        emit("RC_UNHEALTHY");
    }

    if (hasFault(faults, SafetyFault::ThrottleNotLow))
    {
        emit("THROTTLE_NOT_LOW");
    }

    if (hasFault(faults, SafetyFault::ImuInvalid))
    {
        emit("IMU_INVALID");
    }

    if (hasFault(faults, SafetyFault::ImuNotCalibrated))
    {
        emit("IMU_NOT_CALIBRATED");
    }

    if (hasFault(faults, SafetyFault::ImuDataStale))
    {
        emit("IMU_STALE");
    }

    if (hasFault(faults, SafetyFault::ArmSwitchNotActive))
    {
        emit("ARM_SWITCH_OFF");
    }
}

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
        // Barometer snapshot
        // ====================================================

        BMP280::Measurements barometer{};

        const bool barometerValid =
            sensorManager.readBarometer(barometer);

        // ====================================================
        // Summary flags
        // ====================================================

        const bool rcOk =
            rcStateValid &&
            rcHealth.connected &&
            rcHealth.healthy &&
            !rcHealth.failsafe;

        const bool imuOk =
            imuValid &&
            imuSnapshot.calibrated;

        const bool gpsOk =
            telemetryState.satellites > 0 ||
            telemetryState.latitude != 0.0f;

        const std::uint32_t uptimeSeconds =
            static_cast<std::uint32_t>(
                time_us_64() / 1000000ULL);

        // ====================================================
        // Header
        // ====================================================

        printf("\n");
        printf("================================================================\n");
        printf(
            " FLIGHT CONTROLLER   uptime %5lu s   %s\n",
            static_cast<unsigned long>(uptimeSeconds),
            safetyStateName(safetySnapshot.state));

        printf("================================================================\n");

        printf(
            " SUMMARY   RC %s   IMU %s   GPS %s   BARO %s   FAILSAFE %s\n",
            okOrDash(rcOk),
            okOrDash(imuOk),
            okOrDash(gpsOk),
            okOrDash(barometerValid),
            yesNo(rcHealth.failsafe || safetySnapshot.rcFailsafe));

        printf("----------------------------------------------------------------\n");

        // ====================================================
        // IMU
        // ====================================================

        printf(
            " IMU       valid %s   calibrated %s   %lu Hz   seq %lu\n",
            yesNo(imuValid),
            yesNo(imuSnapshot.calibrated),
            static_cast<unsigned long>(samplesPerSecond),
            static_cast<unsigned long>(currentSequence));

        if (imuValid)
        {
            printf(
                "           accel     X %8.4f   Y %8.4f   Z %8.4f g\n",
                imuSnapshot.state.accelX,
                imuSnapshot.state.accelY,
                imuSnapshot.state.accelZ);

            printf(
                "           gyro      X %8.3f   Y %8.3f   Z %8.3f deg/s\n",
                imuSnapshot.state.gyroX,
                imuSnapshot.state.gyroY,
                imuSnapshot.state.gyroZ);
        }

        printf(
            "           attitude  roll %7.2f   pitch %7.2f   yaw %7.2f deg\n",
            flightState.attitudeRoll,
            flightState.attitudePitch,
            flightState.attitudeYaw);

        printf(
            "           calib %u/%u   timestamp %llu us\n",
            static_cast<unsigned>(
                imuCalibration.getSampleCount()),
            static_cast<unsigned>(
                imuCalibration.getRequiredSampleCount()),
            static_cast<unsigned long long>(
                imuSnapshot.state.timestampUs));

        // ====================================================
        // RC
        // ====================================================

        printf("\n");

        printf(
            " RC        connected %s   healthy %s   failsafe %s   valid %s\n",
            yesNo(rcHealth.connected),
            yesNo(rcHealth.healthy),
            yesNo(rcHealth.failsafe),
            yesNo(rcStateValid));

        printf(
            "           frames %lu   %lu fps   age ",
            static_cast<unsigned long>(rcHealth.totalFrames),
            static_cast<unsigned long>(rcHealth.framesLastSecond));

        if (rcHealth.lastFrameTimeUs == 0)
        {
            printf("NEVER");
        }
        else
        {
            printf(
                "%llu us",
                static_cast<unsigned long long>(
                    rcHealth.frameAgeUs));
        }

        printf(
            "   bytes %lu\n",
            static_cast<unsigned long>(rcHealth.totalBytes));

        printf(
            "           channels  %4u %4u %4u %4u | %4u %4u %4u %4u\n",
            rcState.rawChannels[0],
            rcState.rawChannels[1],
            rcState.rawChannels[2],
            rcState.rawChannels[3],
            rcState.rawChannels[4],
            rcState.rawChannels[5],
            rcState.rawChannels[6],
            rcState.rawChannels[7]);

        printf(
            "                     %4u %4u %4u %4u | %4u %4u %4u %4u\n",
            rcState.rawChannels[8],
            rcState.rawChannels[9],
            rcState.rawChannels[10],
            rcState.rawChannels[11],
            rcState.rawChannels[12],
            rcState.rawChannels[13],
            rcState.rawChannels[14],
            rcState.rawChannels[15]);

        printf(
            "           sticks    roll %7.3f   pitch %7.3f   yaw %7.3f   throttle %7.3f\n",
            rcState.roll,
            rcState.pitch,
            rcState.yaw,
            rcState.throttle);

        printf(
            "           arm %s   beeper %s   calib %s   mode %s   frame %lu\n",
            onOff(rcState.arm),
            onOff(rcState.beeper),
            onOff(rcState.calibration),
            flightModeName(rcState.flightMode),
            static_cast<unsigned long>(rcState.frameCounter));

        printf(
            "           timestamp %llu us\n",
            static_cast<unsigned long long>(
                rcState.lastUpdateUs));

        // ====================================================
        // Safety
        // ====================================================

        printf("\n");

        printf(
            " SAFETY    state %s   valid %s\n",
            safetyStateName(safetySnapshot.state),
            yesNo(safetyValid));

        printf(
            "           rc valid %s   failsafe %s   healthy %s\n",
            yesNo(safetySnapshot.rcValid),
            yesNo(safetySnapshot.rcFailsafe),
            yesNo(safetySnapshot.rcHealthy));

        printf(
            "           throttle low %s   arm request %s\n",
            yesNo(safetySnapshot.throttleLow),
            yesNo(safetySnapshot.armRequested));

        printf(
            "           imu valid %s   calibrated %s   fresh %s\n",
            yesNo(safetySnapshot.imuValid),
            yesNo(safetySnapshot.imuCalibrated),
            yesNo(safetySnapshot.imuDataFresh));

        printf("           faults ");
        printFaultList(safetySnapshot.faults);
        printf("\n");

        // ====================================================
        // GPS
        // ====================================================

        printf("\n");

        printf(
            " GPS       fix %s   %u satellites   bytes %lu\n",
            yesNo(gpsOk),
            static_cast<unsigned>(telemetryState.satellites),
            static_cast<unsigned long>(gps.getReceivedBytes()));

        printf(
            "           lat %10.7f   lon %10.7f\n",
            telemetryState.latitude,
            telemetryState.longitude);

        printf(
            "           alt %7.2f m   speed %5.2f m/s   heading %5.1f deg\n",
            telemetryState.gpsAltitude,
            telemetryState.groundSpeed,
            telemetryState.gpsHeading);

        // ====================================================
        // Barometer
        // ====================================================

        printf("\n");

        if (barometerValid)
        {
            printf(
                " BARO      valid YES   temp %5.2f C   pressure %8.2f hPa   alt %7.2f m\n",
                barometer.temperatureC,
                barometer.pressureHpa,
                barometer.altitudeM);
        }
        else
        {
            printf(" BARO      valid NO\n");
        }

        // ====================================================
        // Flight control
        // ====================================================

        printf("\n");

        printf(
            " CONTROL   armed %s   failsafe %s   throttle %6.3f\n",
            yesNo(flightState.armed),
            yesNo(flightState.failsafe),
            flightState.throttle);

        printf(
            "           target rate  roll %6.1f   pitch %6.1f   yaw %6.1f deg/s\n",
            flightState.targetRollRate,
            flightState.targetPitchRate,
            flightState.targetYawRate);

        printf(
            "           motors  M0 %6.3f   M1 %6.3f   M2 %6.3f   M3 %6.3f\n",
            flightState.motors[0],
            flightState.motors[1],
            flightState.motors[2],
            flightState.motors[3]);

        // ====================================================
        // System
        // ====================================================

        printf("\n");

        printf(
            " SYSTEM    heap free %lu bytes\n",
            static_cast<unsigned long>(
                xPortGetFreeHeapSize()));

        printf("================================================================\n");

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
    printf("================================================================\n");
    printf("                 RASPBERRY PI PICO 2 FC\n");
    printf("================================================================\n");

    // ========================================================
    // I2C
    // ========================================================

    printf("\n[1/5] I2C\n");

    if (!i2c.initialize())
    {
        printf(
            "      ERROR: I2C initialization failed.\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf(
        "      SDA GPIO%u   SCL GPIO%u   %lu Hz\n",
        static_cast<unsigned>(
            I2C_SDA_PIN),
        static_cast<unsigned>(
            I2C_SCL_PIN),
        static_cast<unsigned long>(
            I2C_BAUDRATE));

    // ========================================================
    // Sensors
    // ========================================================

    printf("\n[2/5] Sensors\n");

    if (!sensorManager.initialize())
    {
        printf(
            "      ERROR: IMU is not available.\n");

        printf(
            "      Check the I2C wiring and sensor power.\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    // ========================================================
    // ELRS
    // ========================================================

    printf("\n[3/5] ELRS\n");

    if (!elrs.initialize())
    {
        printf(
            "      ERROR: ELRS initialization failed.\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf(
        "      UART1   RX GPIO%u   TX GPIO%u   %lu baud\n",
        static_cast<unsigned>(UART_RX_PIN),
        static_cast<unsigned>(UART_TX_PIN),
        static_cast<unsigned long>(CRSF_BAUDRATE));

    // ========================================================
    // Motors
    // ========================================================

    printf("\n[4/5] Motors\n");

    for (uint8_t i = 0;
         i < MOTOR_COUNT;
         ++i)
    {
        MOTORS[i]->initialize();

        MOTORS[i]->setSpeed(1000);
    }

    printf(
        "      M0 GPIO%u   M1 GPIO%u   M2 GPIO%u   M3 GPIO%u\n",
        static_cast<unsigned>(MOTOR_PINS[0]),
        static_cast<unsigned>(MOTOR_PINS[1]),
        static_cast<unsigned>(MOTOR_PINS[2]),
        static_cast<unsigned>(MOTOR_PINS[3]));

    // ========================================================
    // RC task
    // ========================================================

    printf("\n[5/5] Tasks\n");

    printf("      RC            ");

    if (!rcTask.start(
            "RC",
            1024,
            6))
    {
        printf("FAILED\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf("OK\n");

    // ========================================================
    // IMU task
    // ========================================================

    printf("      IMU           ");

    if (!imuTask.start())
    {
        printf("FAILED\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf("OK\n");

    // ========================================================
    // Safety task
    // ========================================================

    printf("      Safety        ");

    if (!safetyTask.start(
            "Safety",
            1024,
            7))
    {
        printf("FAILED\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf("OK\n");

    // ========================================================
    // Flight control task
    // ========================================================

    printf("      FlightControl ");

    if (!flightControlTask.start(
            "FlightControl",
            2048,
            6))
    {
        printf("FAILED\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf("OK\n");

    // ========================================================
    // Telemetry task
    // ========================================================

    printf("      Telemetry     ");

    if (!telemetryTask.start(
            "Telemetry",
            2048,
            4))
    {
        printf("FAILED\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf("OK\n");

    // ========================================================
    // Diagnostics task
    // ========================================================

    printf("      Diagnostics   ");

    if (xTaskCreate(
            diagnosticsTask,
            "Diagnostics",
            2048,
            nullptr,
            2,
            nullptr) != pdPASS)
    {
        printf("FAILED\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }

    printf("OK\n");

    // ========================================================
    // Scheduler
    // ========================================================

    printf("\n");
    printf("================================================================\n");
    printf(" System ready. Diagnostics run at 1 Hz.\n");
    printf("================================================================\n");

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
