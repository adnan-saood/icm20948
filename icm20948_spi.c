#include <string.h>
#include "driver/spi_master.h"

#include "icm20948.h"
#include "icm20948_spi.h"


ICM_20948_Status_e ICM_20948_internal_write_spi(uint8_t reg, uint8_t *data, uint32_t len, void *handle)
{	
    uint8_t length = len + 1;
    uint8_t tx_buffer[length];
    /* setup data trasmit buffer */
    tx_buffer[0] = ((reg & 0x7F) | 0x00);
    memcpy(tx_buffer+1, data, len);

    spi_transaction_t trans_desc = {
        .flags = 0,
        .tx_buffer = tx_buffer,
        .length = length*8
    };
    if (spi_device_polling_transmit((spi_device_handle_t *)handle, &trans_desc) != ESP_OK)
    {
        return ICM_20948_Stat_Err;
    }
    return ICM_20948_Stat_Ok;
}


ICM_20948_Status_e ICM_20948_internal_read_spi(uint8_t reg, uint8_t *buff, uint32_t len, void *handle)
{
    uint8_t length = len + 1;
    uint8_t tx_buffer[length];    
    uint8_t rx_buffer[length];
    /* setup data buffers */
    memset(tx_buffer+1, 0x0, len);
    tx_buffer[0] = ((reg & 0x7F) | 0x80);
	
	spi_transaction_t trans_desc = {
            .flags = 0,
            .tx_buffer = tx_buffer,
            .rxlength = length*8,
			.rx_buffer = rx_buffer,
            .length = length*8
    };
    if (spi_device_polling_transmit((spi_device_handle_t *)handle, &trans_desc) != ESP_OK)
    {
        return ICM_20948_Stat_Err;
    }
    /* copy received data back to buff */
    memcpy(buff, rx_buffer+1, len);
  	return ICM_20948_Stat_Ok;
}

/* setup a default SPI-serif for a single-device use. If someone wants to use mutliple ICM-20948, the
   a serif for each device has to be implemented. */
ICM_20948_Serif_t default_serif = {
    ICM_20948_internal_write_spi,
    ICM_20948_internal_read_spi,
    NULL,
};


void ICM_20948_init_spi(ICM_20948_Device_t *icm_device, spi_device_handle_t *handle)
{
    default_serif.user = *handle;
    ICM_20948_init_struct(icm_device);
    ICM_20948_link_serif(icm_device, &default_serif);
}


