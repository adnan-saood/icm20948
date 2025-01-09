#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"

#include "driver/spi_master.h"

#include "icm20948.h"
#include "icm20948_spi.h"

#define TAG "spi_agmt"

spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_SPI_MASTER_MISO,
        .mosi_io_num = CONFIG_SPI_MASTER_MOSI,
        .sclk_io_num = CONFIG_SPI_MASTER_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 512 * 8 /* 4095 bytes is the max size of data that can be sent because of hardware limitations */
};

spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4000000,  			/* Clock out at 4 MHz */ 
        .mode = 0,                  			/* SPI mode 0 */
        .spics_io_num = CONFIG_SPI_MASTER_CS, 	/* This field is used to specify the GPIO pin that is to be used as CS' */
        .queue_size = 1             			/* We want to be able to queue 7 transactions at a time */
};


void print_agmt(ICM_20948_AGMT_t agmt) {
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
	spi_device_handle_t spi;

	/* setup SPI bus and add SPI device */
	ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi));
	ICM_20948_init_spi(&icm, &spi);

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

	/* Set Gyro and Accelerometer to a particular sample mode */
	// optiona: ICM_20948_Sample_Mode_Continuous. ICM_20948_Sample_Mode_Cycled
	ICM_20948_set_sample_mode(&icm, sensors, ICM_20948_Sample_Mode_Continuous); 

	/* Set full scale ranges for both acc and gyr */
	ICM_20948_fss_t myfss;
	myfss.a = gpm2;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
	myfss.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
	ICM_20948_set_full_scale(&icm, sensors, myfss);

	/* Set up DLPF configuration */
	ICM_20948_dlpcfg_t myDLPcfg;
	myDLPcfg.a = acc_d473bw_n499bw;
	myDLPcfg.g = gyr_d361bw4_n376bw5;
	ICM_20948_set_dlpf_cfg(&icm, sensors, myDLPcfg);

	/* Choose whether or not to use DLPF */
	ICM_20948_enable_dlpf(&icm, ICM_20948_Internal_Acc, false);
	ICM_20948_enable_dlpf(&icm, ICM_20948_Internal_Gyr, false);

	/* Now wake the sensor up */
	ICM_20948_sleep(&icm, false);
	ICM_20948_low_power(&icm, false);

    /* loop */
    while(1)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);

		ICM_20948_AGMT_t agmt;
		if (ICM_20948_get_agmt(&icm, &agmt) == ICM_20948_Stat_Ok) {
			print_agmt(agmt);
		} else {
			ESP_LOGE(TAG, "Uh oh");
		}        
    }
}
