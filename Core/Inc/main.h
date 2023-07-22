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
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOC
#define GNSS_RESET_Pin GPIO_PIN_0
#define GNSS_RESET_GPIO_Port GPIOC
#define IMU_NSS_Pin GPIO_PIN_4
#define IMU_NSS_GPIO_Port GPIOA
#define BARO_INT_Pin GPIO_PIN_12
#define BARO_INT_GPIO_Port GPIOB
#define SENSOR_INT_Pin GPIO_PIN_13
#define SENSOR_INT_GPIO_Port GPIOB
#define SX1280_INT_Pin GPIO_PIN_14
#define SX1280_INT_GPIO_Port GPIOB
#define SX1280_NSS_1_Pin GPIO_PIN_15
#define SX1280_NSS_1_GPIO_Port GPIOB
#define SX1280_NSS_2_Pin GPIO_PIN_6
#define SX1280_NSS_2_GPIO_Port GPIOC
#define GNSS_EN_Pin GPIO_PIN_8
#define GNSS_EN_GPIO_Port GPIOA
#define GPSS_WAKE_UP_Pin GPIO_PIN_15
#define GPSS_WAKE_UP_GPIO_Port GPIOA
#define GNSS_ON_OFF_Pin GPIO_PIN_10
#define GNSS_ON_OFF_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
