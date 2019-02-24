/*
 * tasks.c
 *
 *  Created on: 26 mar 2018
 *      Author: marek
 */

#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "lwip.h"
#include "usb_device.h"

#include "nrf24l01.h"
#include "nrfconfig.h"
#include "stm32_adafruit_sd.h"
#include "vs1003.h"

#include "pcf8574.h"

#define MUSIC_MSG_SIZE uint8_t[128]

nrfConfig_t nrfConfig =
{
	.channel = 15,
	.payloadSize = 32,
	.speed = NRF_SPEED_250K,
	.outPower = NRF_POWER_0DBM,
	.autoAck = 1,
	.retriesCount = 10,
	.retriesDelay = 1000,
	.addressWidth = 5,
	.address = { 192, 168, 5, 70, 0 },
	.enableCrc = 1,
	.crc = NRF_CRC8,
};

void initNrf24l01(nrfConfig_t *config);
void initSdCard();

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;

extern osMessageQId nrf24l01QueueHandle;
extern osMutexId nrfSdcardSpiMutexHandle;
extern osMutexId nrf24l01MutexHandle;
extern osMutexId sdCardMutexHandle;
extern osSemaphoreId nrf24l01IrqBinarySemHandle;
extern osSemaphoreId vs1003BinarySemHandle;
extern osSemaphoreId vs1003DReqBinarySemHandle;
extern osSemaphoreId playSdCardConsumerBufferBinarySemHandle;
extern osSemaphoreId playSdCardProducerBufferBinarySemHandle;

uint8_t MusicSdCardBuf[128];

void initNrf24l01(nrfConfig_t *config)
{
	nrf24l01DataRate_t speed;
	nrf24l01OutputPower_t power;
	switch (config->speed)
	{
	case NRF_SPEED_250K:
		speed = nrf24l01DataRate250k;
		break;
	case NRF_SPEED_1M:
		speed = nrf24l01DataRate1M;
		break;
	default:
		speed = nrf24l01DataRate2M;
		break;
	}

	switch (config->outPower)
	{
	case NRF_POWER_M18DBM:
		power = nrf24l01OutputPowerM18dBm;
		break;
	case NRF_POWER_M12DBM:
		power = nrf24l01OutputPowerM12dBm;
		break;
	case NRF_POWER_M6DBM:
		power = nrf24l01OutputPowerM6dBm;
		break;
	default:
		power = nrf24l01OutputPower0dBm;
		break;
	}

	nrf24l01Init(config->channel, config->payloadSize);
	nrf24l01SetRF(speed, power);
	nrf24l01SetMyAddress(config->address);
	nrf24l01PowerUpRx();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//  if(GPIO_Pin == WIRELESS_IRQ_EXTI_IRQn)
	{
		osSemaphoreRelease(nrf24l01IrqBinarySemHandle);
	}
//  else
	{
		osSemaphoreRelease(vs1003DReqBinarySemHandle);
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	osSemaphoreRelease(vs1003BinarySemHandle);
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
	/* init code for FATFS */
	MX_FATFS_Init();

	/* init code for LWIP */
	MX_LWIP_Init();

	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();

	osMutexWait(sdCardMutexHandle, osWaitForever);
	initSdCard();
	osMutexRelease(sdCardMutexHandle);

	initNrf24l01(&nrfConfig);
	char testNrf[128] = "testNrf";

	/* Infinite loop */
	for (;;)
	{
		if (osMessagePut(nrf24l01QueueHandle, (uint32_t) testNrf, 0) != osOK)
		{
		}
		osDelay(1000);
	}

	osMutexWait(sdCardMutexHandle, osWaitForever);
	FATFS_UnLinkDriver(USERPath);
	osMutexRelease(sdCardMutexHandle);
}

/* StartNrf24l01Task function */
void StartNrf24l01Task(void const * argument)
{
	if (1)
	{
		vTaskDelete( NULL);
		return;
	}
	uint8_t dataIn[32];
	for (;;)
	{
		if (osSemaphoreWait(nrf24l01IrqBinarySemHandle, portMAX_DELAY) == osOK)
		{
			HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_RESET);
			osMutexWait(nrf24l01MutexHandle, osWaitForever);
			while (nrf24l01DataReady())
			{
				nrf24l01GetData(dataIn);
			}
			osMutexRelease(nrf24l01MutexHandle);
			HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_SET);
		}
	}
}

/* StartNrfSenderTask function */
void StartNrfSenderTask(void const * argument)
{
	if (1)
	{
		vTaskDelete( NULL);
		return;
	}
	osEvent event;
	char *msg;
	uint8_t wait;
	nrf24l01TransmitStatus_t transmissionStatus;

	uint8_t Addresses[] =
	{ 192, 168, 5, 72, 0 };

	for (;;)
	{
		event = osMessageGet(nrf24l01QueueHandle, osWaitForever);

		if (event.status == osEventMessage)
		{
			msg = (char *) event.value.p;
			HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_RESET);
			osMutexWait(nrf24l01MutexHandle, osWaitForever);
			nrf24l01SetTxAddress(Addresses);
			nrf24l01Transmit((uint8_t *) msg);
			wait = 0;
			do
			{
				osDelay(wait++);
				transmissionStatus = nrf24l01GetTransmissionStatus();
			} while (transmissionStatus == nrf24l01TransmitStatusSending);
			nrf24l01PowerUpRx();
			osMutexRelease(nrf24l01MutexHandle);
			HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_SET);
		}
		else
		{
		}
	}
}

/* StartPlayMusicTask function */
void StartPlayMusicTask(void const * argument)
{
	VS1003_Start();
	VS1003_SetVolume(0x50);

	while (1)
	{
		SCI_ChipSelect(RESET);
		SDI_ChipSelect(SET);

		osSemaphoreRelease(playSdCardConsumerBufferBinarySemHandle);
		osSemaphoreWait(playSdCardProducerBufferBinarySemHandle, portMAX_DELAY);
		for (int i = 0; i < 4; i++)
		{
			if (HAL_GPIO_ReadPin(DREQ_PORT, DREQ_PIN) == 0)
			{
				osSemaphoreWait(vs1003DReqBinarySemHandle, portMAX_DELAY);
			}
			HAL_SPI_Transmit_DMA(&hspi1, &MusicSdCardBuf[i * 32], 32);
			osSemaphoreWait(vs1003BinarySemHandle, portMAX_DELAY);
		}
	}
}

/* StartPlayMusicSDCardTask function */
void StartPlayMusicSDCardTask(void const * argument)
{
	FRESULT res;
	uint32_t bytesread;
	uint8_t buf[128];

	while (1)
	{
		osMutexWait(sdCardMutexHandle, osWaitForever);
		if (f_open(&USERFile, "1.mp3", FA_READ) != FR_OK)
		{
			initSdCard();
			osMutexRelease(sdCardMutexHandle);
			continue;
		}
		osMutexRelease(sdCardMutexHandle);

		do
		{
			osMutexWait(sdCardMutexHandle, osWaitForever);
			res = f_read(&USERFile, buf, sizeof(buf), (UINT*) &bytesread);
			osMutexRelease(sdCardMutexHandle);
			if (res != FR_OK)
			{
				break;
			}
			osSemaphoreWait(playSdCardConsumerBufferBinarySemHandle, portMAX_DELAY);
			memcpy(MusicSdCardBuf, buf, sizeof(MusicSdCardBuf));
			osSemaphoreRelease(playSdCardProducerBufferBinarySemHandle);
		} while (bytesread);

		osMutexWait(sdCardMutexHandle, osWaitForever);
		f_close(&USERFile);
		osMutexRelease(sdCardMutexHandle);
	}
}

void initSdCard()
{
	while(1)
	{
		if(f_mount(&USERFatFS, (TCHAR const*) USERPath, 0) == FR_OK)
		{
			return;
		}
		/* FatFs Initialization Error */
		Error_Handler();
	}
}
