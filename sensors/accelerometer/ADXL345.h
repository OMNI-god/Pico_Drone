#pragma once
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

namespace ACL
{
    class ADXL
    {
    private:
        // ADXL345 constants
        static constexpr uint8_t ADDR = 0x53;
        static constexpr uint8_t REG_DEVID = 0x00;
        static constexpr uint8_t REG_POWER_CTL = 0x2D;
        static constexpr uint8_t REG_DATA_FORMAT = 0x31;
        static constexpr uint8_t REG_DATAX0 = 0x32;

        // I2C configuration
        i2c_inst_t *I2C_PORT;
        int SDA_PIN, SCL_PIN, BAUD;

        // Calibration offsets (g)
        float ox = 0, oy = 0, oz = 0;

        // Filter state
        float lpf_ax = 0, lpf_ay = 0, lpf_az = 0;

        // Helpers
        void write_byte(uint8_t reg, uint8_t data);
        void read_bytes(uint8_t reg, uint8_t *buffer, uint8_t len);
        void calibrate();

    public:
        ADXL(i2c_inst_t *port, int sda, int scl, int baud = 400000);
        ~ADXL();
        void init();

        void read_raw(int16_t *x, int16_t *y, int16_t *z);
        void read_accel(float *ax, float *ay, float *az);
        void read_filtered(float *ax, float *ay, float *az);
        void get_angles(float *roll, float *pitch);
    };
}
