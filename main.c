#include <stdio.h>
#include "temp_sensor.h"
#include "i2c.h"

int main(void) {
    int status;
    float temperature;

    /* Initialize the I2C interface. */
    status = i2c_init();
    if (status != 0) {
        printf("I2C initialization failed!\n");
        return 1;
    }

    /* Initialize the temperature sensor. */
    status = temp_sensor_init();
    if (status != 0) {
        printf("Temperature sensor initialization failed!\n");
        return 1;
    }

    /* Read the temperature from the sensor. */
    status = temp_sensor_read(&temperature);
    if (status != 0) {
        printf("Failed to read temperature!\n");
        return 1;
    }

    /* Print the measured temperature. */
    printf("Measured temperature: %.2f °C\n", temperature);

    return 0;
}
