#ifndef _ICM_20948_I2C_H_
#define _ICM_20948_I2C_H_

#include "driver/i2c.h"

typedef struct
{
	i2c_port_t i2c_port;
	uint8_t i2c_addr;
} ICM_20948_Config_i2c_t;


void ICM_20948_init_i2c(ICM_20948_Device_t *device, ICM_20948_Config_i2c_t *config);

/* these functions are exposed in order to make a custom setup of a serif_t possible */
ICM_20948_Status_e ICM_20948_internal_write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user);
ICM_20948_Status_e ICM_20948_internal_read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user);

#endif