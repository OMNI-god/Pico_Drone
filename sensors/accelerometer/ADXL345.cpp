#include "ADXL345.h"
#include <math.h>
#include "pico/stdlib.h"
#include <stdio.h>

using namespace ACL;

#define LPF_FACTOR 0.2f   // Low-pass filter smoothing factor
#define SCALE_G     0.0039f // ADXL345 10-bit sensitivity

ADXL::ADXL(i2c_inst_t* port, int sda, int scl, int baud)
{
    I2C_PORT = port;
    SDA_PIN = sda;
    SCL_PIN = scl;
    BAUD = baud;

    i2c_init(I2C_PORT, BAUD);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    init();
    calibrate();
}

void ADXL::write_byte(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    i2c_write_blocking(I2C_PORT, ADDR, buf, 2, false);
}

void ADXL::read_bytes(uint8_t reg, uint8_t* buffer, uint8_t len)
{
    i2c_write_blocking(I2C_PORT, ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ADDR, buffer, len, false);
}

void ADXL::init()
{
    uint8_t devid;
    read_bytes(REG_DEVID, &devid, 1);

    if (devid != 0xE5)
    {
        printf("ADXL345 ERROR: Wrong device ID: %02X\n", devid);
    }

    write_byte(REG_POWER_CTL, 0x08); // Measurement mode
    write_byte(REG_DATA_FORMAT, 0x00); // ±2g, 10-bit mode

    sleep_ms(20);
}

void ADXL::read_raw(int16_t* x, int16_t* y, int16_t* z)
{
    uint8_t b[6];
    read_bytes(REG_DATAX0, b, 6);

    *x = (int16_t)(b[1] << 8 | b[0]);
    *y = (int16_t)(b[3] << 8 | b[2]);
    *z = (int16_t)(b[5] << 8 | b[4]);
}

void ADXL::read_accel(float* ax, float* ay, float* az)
{
    int16_t x, y, z;
    read_raw(&x, &y, &z);

    *ax = (x * SCALE_G) - ox;
    *ay = (y * SCALE_G) - oy;
    *az = (z * SCALE_G) - oz;
}

void ADXL::read_filtered(float* ax, float* ay, float* az)
{
    float rx, ry, rz;
    read_accel(&rx, &ry, &rz);

    // Real world low-pass filter (exponential smoothing)
    lpf_ax = (LPF_FACTOR * rx) + (1 - LPF_FACTOR) * lpf_ax;
    lpf_ay = (LPF_FACTOR * ry) + (1 - LPF_FACTOR) * lpf_ay;
    lpf_az = (LPF_FACTOR * rz) + (1 - LPF_FACTOR) * lpf_az;

    *ax = lpf_ax;
    *ay = lpf_ay;
    *az = lpf_az;
}

void ADXL::calibrate()
{
    printf("Calibrating ADXL345... keep flat & still\n");

    float sx = 0, sy = 0, sz = 0;

    for (int i = 0; i < 200; i++)
    {
        float ax, ay, az;
        read_accel(&ax, &ay, &az);

        sx += ax;
        sy += ay;
        sz += az;

        sleep_ms(10);
    }

    ox = sx / 200.0f;
    oy = sy / 200.0f;
    
    // Z should read +1g at rest
    oz = (sz / 200.0f) - 1.0f;

    printf("Calibration done.\nOffsets: %.3f %.3f %.3f\n", ox, oy, oz);
}

void ADXL::get_angles(float* roll, float* pitch)
{
    float ax, ay, az;
    read_filtered(&ax, &ay, &az);

    *roll  = atan2(ay, az) * 57.2958f;         // degrees
    *pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 57.2958f;
}
