/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define USB_INT_Pin GPIO_PIN_0
#define USB_INT_GPIO_Port GPIOC
#define ONE_WIRE_Pin GPIO_PIN_4
#define ONE_WIRE_GPIO_Port GPIOA
#define USB_OTG_POWER_Pin GPIO_PIN_4
#define USB_OTG_POWER_GPIO_Port GPIOC
#define FLASH_CS_Pin GPIO_PIN_0
#define FLASH_CS_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define VS1003_XDCS_Pin GPIO_PIN_7
#define VS1003_XDCS_GPIO_Port GPIOE
#define VS1003_DREQ_Pin GPIO_PIN_8
#define VS1003_DREQ_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOE
#define VS1003_XRES_Pin GPIO_PIN_10
#define VS1003_XRES_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOE
#define VS1003_XCS_Pin GPIO_PIN_12
#define VS1003_XCS_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOE
#define LED4_Pin GPIO_PIN_14
#define LED4_GPIO_Port GPIOE
#define WIRELESS_CS_Pin GPIO_PIN_13
#define WIRELESS_CS_GPIO_Port GPIOD
#define WIRELESS_CE_Pin GPIO_PIN_14
#define WIRELESS_CE_GPIO_Port GPIOD
#define WIRELESS_IRQ_Pin GPIO_PIN_15
#define WIRELESS_IRQ_GPIO_Port GPIOD
#define WIRELESS_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_S1_Pin GPIO_PIN_6
#define BUTTON_S1_GPIO_Port GPIOC
#define BUTTON_S2_Pin GPIO_PIN_7
#define BUTTON_S2_GPIO_Port GPIOC
#define BUTTON_S3_Pin GPIO_PIN_8
#define BUTTON_S3_GPIO_Port GPIOC
#define BUTTON_S4_Pin GPIO_PIN_9
#define BUTTON_S4_GPIO_Port GPIOC
#define MCO_ETH_Pin GPIO_PIN_8
#define MCO_ETH_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_9
#define RS485_TX_GPIO_Port GPIOA
#define RS485_RX_Pin GPIO_PIN_10
#define RS485_RX_GPIO_Port GPIOA
#define RS232_RX2_Pin GPIO_PIN_2
#define RS232_RX2_GPIO_Port GPIOD
#define RS485_DIR_Pin GPIO_PIN_4
#define RS485_DIR_GPIO_Port GPIOD
#define RS232_TX1_Pin GPIO_PIN_5
#define RS232_TX1_GPIO_Port GPIOD
#define RS232_RX1_Pin GPIO_PIN_6
#define RS232_RX1_GPIO_Port GPIOD

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
