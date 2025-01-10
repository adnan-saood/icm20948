#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"

#include "icm20948.h"
#include "icm20948_i2c.h"

#define TAG "i2c_dmp_quat9_orientation"

/* i2c bus configuration */
i2c_config_t conf = {
	.mode = I2C_MODE_MASTER,
	.sda_io_num = (gpio_num_t) CONFIG_I2C_MASTER_SDA,
	.sda_pullup_en = GPIO_PULLUP_ENABLE,
	.scl_io_num = (gpio_num_t) CONFIG_I2C_MASTER_SCL,
	.scl_pullup_en = GPIO_PULLUP_ENABLE,
	.master.clk_speed = 400000,
	.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
};

/* ICM 20948 configuration */
ICM_20948_Config_i2c_t icm_config = {
	.i2c_port = I2C_NUM_0,
	.i2c_addr = ICM_20948_I2C_ADDR_AD1
};


void init_dmp(ICM_20948_Device_t *icm)
{
  	bool success = true; // Use success to show if the DMP configuration was successful

  	// Initialize the DMP with defaults.
  	success &= (ICM_20948_init_dmp_sensor_with_defaults(icm) == ICM_20948_Stat_Ok);
	// DMP sensor options are defined in ICM_20948_DMP.h
	//    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
	//    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
	//    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
	//    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
	//    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
	//    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
	//    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
	//    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
	//    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
	//    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
	//    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
	//    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
	//    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
	//    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
	//    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

	// Enable the DMP orientation sensor
	success &= (inv_icm20948_enable_dmp_sensor(icm, INV_ICM20948_SENSOR_ORIENTATION, 1) == ICM_20948_Stat_Ok);

	// Enable any additional sensors / features
	//success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
	//success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
	//success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

	// Configuring DMP to output data at multiple ODRs:
	// DMP is capable of outputting multiple sensor data at different rates to FIFO.
	// Setting value can be calculated as follows:
	// Value = (DMP running rate / ODR ) - 1
	// E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
	success &= (inv_icm20948_set_dmp_sensor_period(icm, DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
	//success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
	//success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
	//success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
	//success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
	//success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
	// Enable the FIFO
	success &= (ICM_20948_enable_FIFO(icm, true) == ICM_20948_Stat_Ok);
	// Enable the DMP
	success &= (ICM_20948_enable_DMP(icm, 1) == ICM_20948_Stat_Ok);
	// Reset DMP
	success &= (ICM_20948_reset_DMP(icm) == ICM_20948_Stat_Ok);
	// Reset FIFO
	success &= (ICM_20948_reset_FIFO(icm) == ICM_20948_Stat_Ok);

	// Check success
	if (success)
	{
		ESP_LOGI(TAG, "DMP enabled!");
	} else {
		ESP_LOGE(TAG, "Enable DMP failed!");
		while (1)
		; // Do nothing more
	}	
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

	/* now the fun with DMP starts */
	init_dmp(&icm);

    while(1)
	{
		// Read any DMP data waiting in the FIFO
		// Note:
		//    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
		//    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
		//    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
		//    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
		//    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
		icm_20948_DMP_data_t data;
		ICM_20948_Status_e status = inv_icm20948_read_dmp_data(&icm, &data);
		/* Was valid data available? */
  		if ((status == ICM_20948_Stat_Ok) || (status == ICM_20948_Stat_FIFOMoreDataAvail)) 
		{
			/* We have asked for orientation data so we should receive Quat9 */
			if ((data.header & DMP_header_bitmap_Quat9) > 0) 
			{
				// Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
				// In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
				// The quaternion data is scaled by 2^30.
				// Scale to +/- 1
				double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
				double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
				double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
				double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
				ESP_LOGI(TAG, "Q1: %f Q2: %f Q3: %f Accuracy: %d", q1, q2, q3, data.Quat9.Data.Accuracy);
			}
		}
		if(status != ICM_20948_Stat_FIFOMoreDataAvail) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
	}	
}
