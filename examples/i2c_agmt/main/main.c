#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"

#include "icm20948.h"
#include "icm20948_i2c.h"

#define TAG "i2c_agmt"

/* i2c bus configuration */
i2c_config_t conf = {
	.mode = I2C_MODE_MASTER,
	.sda_io_num = (gpio_num_t) 9,
	.sda_pullup_en = GPIO_PULLUP_ENABLE,
	.scl_io_num = (gpio_num_t) 3,
	.scl_pullup_en = GPIO_PULLUP_ENABLE,
	.master.clk_speed = 400000,
	.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
};

/* ICM 20948 configuration */
ICM_20948_Config_i2c_t icm_config = {
	.i2c_port = I2C_NUM_0,
	.i2c_addr = ICM_20948_I2C_ADDR_AD1
};


void print_agmt(ICM_20948_AGMT_t agmt)
{
  	ESP_LOGI(TAG, "Acc: [ %d, %d, %d ] Gyr: [ %d, %d, %d ] Mag: [ %d, %d, %d ] Tmp: [ %d ]", 
		agmt.acc.axes.x, agmt.acc.axes.y, agmt.acc.axes.z,
		agmt.gyr.axes.x, agmt.gyr.axes.y, agmt.gyr.axes.z,
		agmt.mag.axes.x, agmt.mag.axes.y, agmt.mag.axes.z,
		agmt.tmp.val
	);
}

void app_main(void)
{
	ICM_20948_Device_t icm;

	/* setup i2c */
	ESP_ERROR_CHECK(i2c_param_config(icm_config.i2c_port, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(icm_config.i2c_port, conf.mode, 0, 0, 0));
	
	/* setup ICM20948 device */
	ICM_20948_init_i2c(&icm, &icm_config);
		
	/* check ID */
    while (ICM_20948_check_id(&icm) != ICM_20948_Stat_Ok)
	{
		ESP_LOGE(TAG, "check id failed");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
	ESP_LOGI(TAG, "check id passed");

	/* check whoami */
	ICM_20948_Status_e stat = ICM_20948_Stat_Err;
	uint8_t whoami = 0x00;
	while ((stat != ICM_20948_Stat_Ok) || (whoami != ICM_20948_WHOAMI))
	{
		whoami = 0x00;
		stat = ICM_20948_get_who_am_i(&icm, &whoami);
		ESP_LOGE(TAG, "whoami does not match (0x %d). Halting...", whoami);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	/* Here we are doing a SW reset to make sure the device starts in a known state */
	ICM_20948_sw_reset(&icm);
	vTaskDelay(250 / portTICK_PERIOD_MS);

	ICM_20948_InternalSensorID_bm sensors = (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr);

	// Set Gyro and Accelerometer to a particular sample mode
	// optiona: ICM_20948_Sample_Mode_Continuous. ICM_20948_Sample_Mode_Cycled
	ICM_20948_set_sample_mode(&icm, sensors, ICM_20948_Sample_Mode_Continuous); 

	// Set full scale ranges for both acc and gyr
	ICM_20948_fss_t myfss;
	myfss.a = gpm2;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
	myfss.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
	ICM_20948_set_full_scale(&icm, sensors, myfss);

	// Set up DLPF configuration
	ICM_20948_dlpcfg_t myDLPcfg;
	myDLPcfg.a = acc_d473bw_n499bw;
	myDLPcfg.g = gyr_d361bw4_n376bw5;
	ICM_20948_set_dlpf_cfg(&icm, sensors, myDLPcfg);

	// Choose whether or not to use DLPF
	ICM_20948_enable_dlpf(&icm, ICM_20948_Internal_Acc, false);
	ICM_20948_enable_dlpf(&icm, ICM_20948_Internal_Gyr, false);

	// Now wake the sensor up
	ICM_20948_sleep(&icm, false);
	ICM_20948_low_power(&icm, false);

    /* loop */
    while(1)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);

		ICM_20948_AGMT_t agmt; // = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0}};
		if (ICM_20948_get_agmt(&icm, &agmt) == ICM_20948_Stat_Ok) {
			print_agmt(agmt);
		} else {
			ESP_LOGE(TAG, "Uh oh");
		}        
    }
}
