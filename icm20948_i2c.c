#include "driver/i2c.h"

#include "icm20948.h"
#include "icm20948_i2c.h"

#define ACK_CHECK_EN   0x1     /* I2C master will check ack from slave */
#define ACK_CHECK_DIS  0x0     /* I2C master will not check ack from slave */


ICM_20948_Status_e ICM_20948_internal_write_i2c(uint8_t reg, uint8_t *data, uint32_t len, void *user)
{
	ICM_20948_Status_e status = ICM_20948_Stat_Ok;
	ICM_20948_Config_i2c_t *args = (ICM_20948_Config_i2c_t*)user;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (args->i2c_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, true);
	i2c_master_write(cmd, data, len, true);
	i2c_master_stop(cmd);
	
	if(i2c_master_cmd_begin(args->i2c_port, cmd, 100 / portTICK_PERIOD_MS) != ESP_OK)
	{
		status = ICM_20948_Stat_Err;
	}
	i2c_cmd_link_delete(cmd);
    return status;
}

ICM_20948_Status_e ICM_20948_internal_read_i2c(uint8_t reg, uint8_t *buff, uint32_t len, void *user)
{
	ICM_20948_Status_e status = ICM_20948_Stat_Ok;
	ICM_20948_Config_i2c_t *args = (ICM_20948_Config_i2c_t*)user;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (args->i2c_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, true);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (args->i2c_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
	i2c_master_read(cmd, buff, len, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);
	
	if(i2c_master_cmd_begin(args->i2c_port, cmd, 100 / portTICK_PERIOD_MS) != ESP_OK)
	{
		status = ICM_20948_Stat_Err;
	}
	i2c_cmd_link_delete(cmd);
    return status;
}

/* default serif */
ICM_20948_Serif_t default_serif = {
    ICM_20948_internal_write_i2c,
    ICM_20948_internal_read_i2c,
    NULL,
};

void ICM_20948_init_i2c(ICM_20948_Device_t *icm_device, ICM_20948_Config_i2c_t *args)
{
	ICM_20948_init_struct(icm_device);
	default_serif.user = (void *) &args;
    ICM_20948_link_serif(icm_device, &default_serif);
}
