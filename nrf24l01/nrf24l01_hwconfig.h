/*
 * nrf24l01_config.h
 *
 *  Created on: 11 gru 2015
 *      Author: marek
 */

#ifndef NRF24L01_HWCONFIG_H_
#define NRF24L01_HWCONFIG_H_

#include "main.h"

/* SPI chip enable pin */
#define NRF24L01_CSN_PORT			WIRELESS_CS_GPIO_Port
#define NRF24L01_CSN_PIN			WIRELESS_CS_Pin

/* Chip enable for transmitting */
#define NRF24L01_CE_PORT			WIRELESS_CE_GPIO_Port
#define NRF24L01_CE_PIN			WIRELESS_CE_Pin

/* Pins configuration */
#define NRF24L01_CE_LOW			HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_RESET)
#define NRF24L01_CE_HIGH			HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_SET)
#define NRF24L01_CSN_LOW			HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_PIN_RESET)
#define NRF24L01_CSN_HIGH			HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_PIN_SET)

#endif /* NRF24L01_HWCONFIG_H_ */
