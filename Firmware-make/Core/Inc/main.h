/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPS_RST_Pin GPIO_PIN_13
#define GPS_RST_GPIO_Port GPIOC
#define GPS_INT_Pin GPIO_PIN_14
#define GPS_INT_GPIO_Port GPIOC
#define MCU_TX_GPS_RX_Pin GPIO_PIN_2
#define MCU_TX_GPS_RX_GPIO_Port GPIOA
#define MCU_RX_GPS_TX_Pin GPIO_PIN_3
#define MCU_RX_GPS_TX_GPIO_Port GPIOA
#define TIMEPULSE_Pin GPIO_PIN_5
#define TIMEPULSE_GPIO_Port GPIOA
#define SAFEBOOT_Pin GPIO_PIN_6
#define SAFEBOOT_GPIO_Port GPIOA
#define USR_LED1_Pin GPIO_PIN_0
#define USR_LED1_GPIO_Port GPIOB
#define USR_LED2_Pin GPIO_PIN_1
#define USR_LED2_GPIO_Port GPIOB
#define EEPROM_WP_Pin GPIO_PIN_12
#define EEPROM_WP_GPIO_Port GPIOB
#define BMP_CS_Pin GPIO_PIN_2
#define BMP_CS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
