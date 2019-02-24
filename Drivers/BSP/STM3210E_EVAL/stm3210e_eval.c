/**
 ******************************************************************************
 * @file    stm3210e_eval.c
 * @author  MCD Application Team
 * @version V7.0.0
 * @date    14-April-2017
 * @brief   This file provides a set of firmware functions to manage Leds,
 *          push-button and COM ports for STM3210E_EVAL
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm3210e_eval.h"
/**
 * @brief BUS variables
 */
#ifdef HAL_SPI_MODULE_ENABLED
uint32_t SpixTimeout = EVAL_SPIx_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */
static SPI_HandleTypeDef heval_Spi;
#endif /* HAL_SPI_MODULE_ENABLED */

/******************************* SPI Routines**********************************/
#ifdef HAL_SPI_MODULE_ENABLED

/**
 * @brief  Initializes SPI HAL.
 */
HAL_StatusTypeDef SPIx_Init(void)
{
	/* DeInitializes the SPI peripheral */
	heval_Spi.Instance = EVAL_SPIx;
	HAL_SPI_DeInit(&heval_Spi);

	/* SPI Config */
	/* SPI baudrate is set to 36 MHz (PCLK2/SPI_BaudRatePrescaler = 72/2 = 36 MHz) */
	heval_Spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	heval_Spi.Init.Direction = SPI_DIRECTION_2LINES;
	heval_Spi.Init.CLKPhase = SPI_PHASE_1EDGE;
	heval_Spi.Init.CLKPolarity = SPI_POLARITY_LOW;
	heval_Spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	heval_Spi.Init.CRCPolynomial = 7;
	heval_Spi.Init.DataSize = SPI_DATASIZE_8BIT;
	heval_Spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	heval_Spi.Init.NSS = SPI_NSS_SOFT;
	heval_Spi.Init.TIMode = SPI_TIMODE_DISABLE;
	heval_Spi.Init.Mode = SPI_MODE_MASTER;

//  SPIx_MspInit(&heval_Spi);

	return (HAL_SPI_Init(&heval_Spi));
}

/**
 * @brief SPI error treatment function
 */
void SPIx_Error(void)
{
	/* De-initialize the SPI communication BUS */
	HAL_SPI_DeInit(&heval_Spi);

	/* Re- Initiaize the SPI communication BUS */
	SPIx_Init();
}
#endif /* HAL_SPI_MODULE_ENABLED */

/**
 * @}
 */

/** @defgroup STM3210E_EVAL_LinkOperations_Functions STM3210E EVAL LinkOperations Functions
 * @{
 */

/*******************************************************************************
 LINK OPERATIONS
 *******************************************************************************/

#define SD_DUMMY_BYTE      0xFF

/******************************** LINK SD Card ********************************/

/**
 * @brief  Initializes the SD Card and put it into StandBy State (Ready for
 *         data transfer).
 */
void SD_IO_Init(void)
{
	GPIO_InitTypeDef gpioinitstruct;
	uint8_t counter;

	/* SD_CS_GPIO and SD_DETECT_GPIO Periph clock enable */
//  SD_CS_GPIO_CLK_ENABLE();
//  SD_DETECT_GPIO_CLK_ENABLE();
//  /* Configure SD_CS_PIN pin: SD Card CS pin */
//  gpioinitstruct.Pin    = SD_CS_PIN;
//  gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
//  gpioinitstruct.Pull   = GPIO_PULLUP;
//  gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
//  HAL_GPIO_Init(SD_CS_GPIO_PORT, &gpioinitstruct);
//
//  /* Configure SD_DETECT_PIN pin: SD Card detect pin */
//  gpioinitstruct.Pin    = SD_DETECT_PIN;
//  gpioinitstruct.Mode   = GPIO_MODE_IT_RISING_FALLING;
//  gpioinitstruct.Pull   = GPIO_PULLUP;
//  HAL_GPIO_Init(SD_DETECT_GPIO_PORT, &gpioinitstruct);
	/*------------Put SD in SPI mode--------------*/
	/* SD SPI Config */
	SPIx_Init();

	/* SD chip select high */
	SD_IO_CSState(1);

	/* Send dummy byte 0xFF, 10 times with CS high */
	/* Rise CS and MOSI for 80 clocks cycles */
	for (counter = 0; counter <= 9; counter++)
	{
		/* Send dummy byte 0xFF */
		SD_IO_WriteByte(SD_DUMMY_BYTE);
	}
}

/**
 * @brief  Set the SD_CS pin.
 * @param  pin value.
 * @retval None
 */
void SD_IO_CSState(uint8_t val)
{
	if (val == 1)
	{
		SD_CS_HIGH();
	}
	else
	{
		SD_CS_LOW();
	}
}

uint8_t SD_IO_WriteByte(uint8_t Data)
{
	uint8_t tmp;
	/* Send the byte */
	SPIx_WriteReadData(&Data, &tmp, 1);
	return tmp;
}

void SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
	/* Send the byte */
	SPIx_WriteReadData(DataIn, DataOut, DataLength);
}

void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_SPI_TransmitReceive(&heval_Spi, (uint8_t*) DataIn, DataOut, DataLength, SpixTimeout);

	/* Check the communication status */
	if (status != HAL_OK)
	{
		/* Execute user timeout callback */
		SPIx_Error();
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

