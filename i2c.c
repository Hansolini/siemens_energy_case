#include "i2c.h"
#include <stdint.h>

/* Register definitions – these match what the sensor code expects. */
#define TMP_SENSOR_ADDR  0x77   // Sensor I2C address
#define TMP_CFG_REG      0x07   // Temperature configuration register
#define MEAS_CFG         0x08   // Measurement configuration register
#define CFG_REG          0x09   // FIFO/interrupt config register

// Temperature data registers (3 bytes)
#define TMP_B2_REG       0x03   // Temperature MSB
#define TMP_B1_REG       0x04
#define TMP_B0_REG       0x05

// Calibration registers (3 bytes)
#define TMP_C0_REG       0x10   // Calibration c0 (MSB)
#define TMP_C0C1_REG     0x11   // High nibble: lower bits of c0; Low nibble: upper bits of c1
#define TMP_C1_REG       0x12   // Calibration c1 (LSB)

static uint8_t sensor_memory[256];
static uint8_t sensor_pointer = 0;

int i2c_init(void) {
    /* Initialize calibration registers */
    sensor_memory[TMP_C0_REG]   = 0x00;
    sensor_memory[TMP_C0C1_REG] = 0x00;
    sensor_memory[TMP_C1_REG]   = 0x64;

    /* Initialize temperature data registers (simulate 25 degrees) */
    sensor_memory[TMP_B2_REG] = 0x20;
    sensor_memory[TMP_B1_REG] = 0x00;
    sensor_memory[TMP_B0_REG] = 0x00;
    
    return 0;
}

int i2c_write(uint8_t address, uint8_t *data, uint8_t len) {
    if (address != TMP_SENSOR_ADDR) {
        // For other addresses, do nothing.
        return 0;
    }
    if (len == 0) {
        return 0;
    }

    if (len == 1) {
        sensor_pointer = data[0];
    } else {
        // First byte is the register address; following bytes are data to write.
        sensor_pointer = data[0];
        for (uint8_t i = 1; i < len; i++) {
            sensor_memory[sensor_pointer++] = data[i];
        }
    }
    return 0;
}

int i2c_read(uint8_t address, uint8_t *data, uint8_t len) {
    if (address != TMP_SENSOR_ADDR) {
        // For any unknown device, return zeros.
        for (uint8_t i = 0; i < len; i++) {
            data[i] = 0;
        }
        return 0;
    }
    for (uint8_t i = 0; i < len; i++) {
        data[i] = sensor_memory[sensor_pointer++];
    }
    return 0;
}
