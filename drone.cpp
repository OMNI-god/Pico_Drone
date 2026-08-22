#include <cstdio>

#include "pico/stdlib.h"

#include "I2c.h"
#include "ICM20948.h"
#include "GY271.h"
#include "BMP280.h"
#include "ADXL345.h"
#include "SensorManager.h"

int main()
{
    stdio_init_all();

    sleep_ms(2000);

    printf("\n");
    printf("==============================\n");
    printf(" SensorManager Test\n");
    printf("==============================\n");

    // --------------------------------------------------
    // I2C
    // --------------------------------------------------

    I2c i2c(
        i2c1,
        400000,
        3, // SDA
        2  // SCL
    );

    if (!i2c.initialize())
    {
        printf("I2C initialization FAILED\n");
        // while (true)
        // {
        //     sleep_ms(1000);
        // }
    }

    printf("I2C initialized\n");
    printf("SDA = GPIO 3\n");
    printf("SCL = GPIO 2\n");

    // --------------------------------------------------
    // Sensors
    // --------------------------------------------------

    ICM20948 imu(i2c);

    GY271 magnetometer(i2c);

    BMP280 barometer(i2c);

    ADXL345 accelerometer(i2c);

    barometer.initialize();

    while (true)
    {
        BMP280::Measurements data{};

        if (barometer.read(data))
        {
            printf(
                "BMP280: %.2f C, %.2f hPa\n",
                data.temperatureC,
                data.pressureHpa);
        }
        else
        {
            printf("BMP280 READ FAILED\n");
        }

        sleep_ms(1000);
    }

    // // --------------------------------------------------
    // // Sensor Manager
    // // --------------------------------------------------

    // SensorManager sensors(
    //     imu,
    //     magnetometer,
    //     barometer,
    //     accelerometer);

    // printf("\nInitializing SensorManager...\n");

    // if (!sensors.initialize())
    // {
    //     printf("\nSensorManager initialization FAILED\n");

    //     while (true)
    //     {
    //         sleep_ms(1000);
    //     }
    // }

    // printf("\nSensorManager initialized successfully!\n");

    // // --------------------------------------------------
    // // Read sensors
    // // --------------------------------------------------

    // SensorManager::SensorData data;

    // while (true)
    // {
    //     if (sensors.read(data))
    //     {
    //         printf("\n--- Sensor Data ---\n");

    //         printf(
    //             "IMU Accel: X=%.3f Y=%.3f Z=%.3f g\n",
    //             data.imu.acceleration.x,
    //             data.imu.acceleration.y,
    //             data.imu.acceleration.z);

    //         printf(
    //             "IMU Gyro:  X=%.3f Y=%.3f Z=%.3f dps\n",
    //             data.imu.gyroscope.x,
    //             data.imu.gyroscope.y,
    //             data.imu.gyroscope.z);

    //         printf(
    //             "MAG:       X=%.3f Y=%.3f Z=%.3f G\n",
    //             data.magnetometer.x,
    //             data.magnetometer.y,
    //             data.magnetometer.z);

    //         printf(
    //             "BARO:      Pressure=%.2f Pa Temp=%.2f C\n",
    //             data.barometer.pressureHpa,
    //             data.barometer.temperatureC);

    //         printf(
    //             "ADXL:      X=%.3f Y=%.3f Z=%.3f g\n",
    //             data.externalAccelerometer.x,
    //             data.externalAccelerometer.y,
    //             data.externalAccelerometer.z);
    //     }
    //     else
    //     {
    //         printf("SensorManager::read() FAILED\n");
    //     }

    //     sleep_ms(100);
    // }
}