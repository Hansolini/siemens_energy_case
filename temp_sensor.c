#include "temp_sensor.h"
#include "i2c.h"

#define TMP_SENSOR_ADDR  0x77   // Default I2C address for DPS310 (SDO pin is not pulled down)
#define TMP_CFG_REG      0x07   // Temperature configuration register
#define MEAS_CFG         0x08   // Sensor operating mode and status
#define CFG_REG          0x09   // Interrupt and FIFO configuration

// Registers for temperature readings (3 bytes)
#define TMP_B2_REG       0x03   // MSB
#define TMP_B1_REG       0x04
#define TMP_B0_REG       0x05

// Calibration registers for temperature coefficients
#define TMP_C0_REG       0x10   // MSB
#define TMP_C0C1_REG     0x11
#define TMP_C1_REG       0x12

// Global calibration coefficients
static float c0 = 0.0f;
static float c1 = 0.0f;

// Temperature scaling factor (single precision)
static const int32_t kT = 524288;  // 2^19

// Sensor initialization
int temp_sensor_init(void)
{
    int status;

    /* 1. Read temperature calibration coefficients */
    uint8_t reg = TMP_C0_REG;
    
    status = i2c_write(TMP_SENSOR_ADDR, &reg, 1);
    if (status != 0) {
        return -1;  // Error setting calibration register address
    }

    uint8_t cal_data[3];
    status = i2c_read(TMP_SENSOR_ADDR, cal_data, 3);
    if (status != 0) {
        return -1;  // Error reading calibration data
    }

    /* Extract calibration registers */
    uint8_t reg0x10 = cal_data[0];
    uint8_t reg0x11 = cal_data[1];
    uint8_t reg0x12 = cal_data[2];

    // Compute c0 (12-bit, two's complement)
    int16_t raw_c0 = ((int16_t)reg0x10 << 4) | (reg0x11 >> 4);
    if (raw_c0 >= 2048) {
        raw_c0 -= 4096;
    }
    c0 = (float)raw_c0;

    // Compute c1 (12-bit, two's complement)
    int16_t raw_c1 = ((reg0x11 & 0x0F) << 8) | reg0x12;
    if (raw_c1 >= 2048) {
        raw_c1 -= 4096;
    }
    c1 = (float)raw_c1;

    /* 2. Set up sensor configuration for temperature measurement */
    uint8_t tmp_cfg[2];
    tmp_cfg[0] = TMP_CFG_REG;
    tmp_cfg[1] &= ~(1 << 7);      // TMP_EXT = 0 (internal sensor)
    tmp_cfg[1] &= ~(0b111 << 4);  // TMP_RATE = 000 (1 Hz)
    tmp_cfg[1] &= ~(0b1111);      // TMP_PRC = 0000 (single measurement)
    
    status = i2c_write(TMP_SENSOR_ADDR, tmp_cfg, 2);
    if (status != 0) {
        return -1;  // Error writing temperature configuration
    }

    /* 3. Set up interrupt and FIFO configuration */
    uint8_t cfg[2];
    cfg[0] = CFG_REG;
    cfg[1] = (1 << 1);  // Enable FIFO

    status = i2c_write(TMP_SENSOR_ADDR, cfg, 2);
    if (status != 0) {
        return -1;  // Error writing FIFO configuration
    }

    return 0;  // Success
}

// Read temperature
int temp_sensor_read(float *temp)
{
    int status;

    // Initiate a temperature measurement
    uint8_t meas_cfg[2] = {MEAS_CFG, 0x02}; // Starts temperature measurement

    status = i2c_write(TMP_SENSOR_ADDR, meas_cfg, 2);
    if (status != 0) {
        return -1;  // Error initiating temperature measurement
    }

    // Set the register pointer to the temperature data (MSB register)
    uint8_t reg = TMP_B2_REG;
    
    status = i2c_write(TMP_SENSOR_ADDR, &reg, 1);
    if (status != 0) {
        return -1;  // Error setting temperature data register address
    }

    // Read raw temperature data
    uint8_t data[3];
    
    status = i2c_read(TMP_SENSOR_ADDR, data, 3);
    if (status != 0) {
        return -1;  // Error reading temperature data
    }

    // Combine data bytes
    uint32_t raw24 = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    int32_t tmp_raw = raw24 >> 4;  // Discard the lower 4 fractional bits

    if (tmp_raw & (1 << 19)) {
        tmp_raw -= (1 << 20);
    }

    // Scale the raw temperature data
    float tmp_raw_sc = (float)tmp_raw/kT;

    // Calculate the compensated temperature
    float tmp_comp = (c0*0.5f) + (c1*tmp_raw_sc);

    // Return calculated temperature
    *temp = tmp_comp;

    return 0;  // Success
}
