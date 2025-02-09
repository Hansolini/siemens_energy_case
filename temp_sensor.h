#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

/**
 * @brief Initialize temperature sensor
 * 
 * @return Zero on success, nonzero on error
 */
int temp_sensor_init(void);

/**
 * @brief Read temperature
 * 
 * @param temp Pointer to temperature output variable
 * @return Zero on success, nonzero on error
 */
int temp_sensor_read(float *temp);

#endif
