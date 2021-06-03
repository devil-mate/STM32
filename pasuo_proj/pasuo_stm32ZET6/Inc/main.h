/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define CAM_Pin GPIO_PIN_11
#define CAM_GPIO_Port GPIOF
#define LED_B_Pin GPIO_PIN_15
#define LED_B_GPIO_Port GPIOF
#define Tx3_Pin GPIO_PIN_10
#define Tx3_GPIO_Port GPIOB
#define Rx3_Pin GPIO_PIN_11
#define Rx3_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOG
#define Tx485_Pin GPIO_PIN_10
#define Tx485_GPIO_Port GPIOC
#define Rx485_Pin GPIO_PIN_11
#define Rx485_GPIO_Port GPIOC
#define Tx5_Pin GPIO_PIN_12
#define Tx5_GPIO_Port GPIOC
#define Rx5_Pin GPIO_PIN_2
#define Rx5_GPIO_Port GPIOD
#define Tx1_Pin GPIO_PIN_6
#define Tx1_GPIO_Port GPIOB
#define Rx1_Pin GPIO_PIN_7
#define Rx1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
