#ifndef I2C_H
#define I2C_H

#include <stdint.h>

/**
 * @brief Initialize I2C pins and registers
 * 
 * @return Zero on success, nonzero on error
 */
int i2c_init(void);

/**
 * @brief Write bytes to I2C bus
 * 
 * @param address Address of I2C device
 * @param data Buffer for data
 * @param len Number of bytes to write
 * @return Zero on success, nonzero on error
 */
int i2c_write(uint8_t address, uint8_t *data, uint8_t len);

/**
 * @brief Read bytes from I2C bus
 * 
 * @param address Address of I2C device
 * @param data Buffer for data
 * @param len Number of bytes to read
 * @return Zero on success, nonzero on error
 */
int i2c_read(uint8_t address, uint8_t *data, uint8_t len);

#endif
