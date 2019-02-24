#include "pcf8574.h"

PCF8574_RESULT pcf8574_init(pcf8574* handle)
{
	handle->address &= 0x07;
	handle->address <<= 1;
	handle->address |= 0x40;
	return PCF8574_OK;
}

PCF8574_RESULT pcf8574_deinit(pcf8574* handle)
{
	return PCF8574_OK;
}

PCF8574_RESULT pcf8574_write(pcf8574* handle, uint8_t val)
{
	if (HAL_I2C_Master_Transmit(handle->i2c, handle->address, &val, 1,
			handle->timeout) != HAL_OK)
	{
		return PCF8574_ERROR;
	}
	return PCF8574_OK;
}

PCF8574_RESULT pcf8574_read(pcf8574* handle, uint8_t* val)
{
	if (HAL_I2C_Master_Receive(handle->i2c, handle->address, val, 1,
			handle->timeout) != HAL_OK)
	{
		return PCF8574_ERROR;
	}
	return PCF8574_OK;
}

/***** usage *****
 *
 *   pcf8574 pcf;

  pcf.address = 0x38;
  pcf.timeout = 1000;
  pcf.i2c     = &hi2c1;
  pcf8574_init(&pcf);
  uint8_t val;
  pcf8574_write(&pcf, &val);
 * /
  	    pcf8574 pcf;

	    pcf.address = 0x38;
	    pcf.timeout = 1000;
	    pcf.i2c     = &hi2c1;

		for (int i=1; i<128 || 1; i++)
	 	{
	 	  /*
	 	   * the HAL wants a left aligned i2c address
	 	   * &hi2c1 is the handle
	 	   * (uint16_t)(i<<1) is the i2c address left aligned
	 	   * retries 2
	 	   * timeout 2
	 	   * /
	        uint8_t val;
	        pcf8574_write(&pcf, &val);
	 	 int result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
	 	  if (result == HAL_OK)
	 	  {
	 		  printf("0x%X", i); // Received an ACK at that address
	 	  }
	 	}

	    if (pcf8574_init(&pcf) != PCF8574_OK) Error_Handler();

	    while (1)
	    {
	        uint8_t val;
	        if (pcf8574_read(&pcf, &val) != PCF8574_OK) Error_Handler();
	        val++;
	        if (pcf8574_write(&pcf, val) != PCF8574_OK) Error_Handler();
	        HAL_Delay(100);
	    }
 */
