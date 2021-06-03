/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f2xx_hal.h"

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
//#define ENABLE_WATCHDOG
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POW4G_Pin GPIO_PIN_0
#define POW4G_GPIO_Port GPIOC
#define POWRADIO_Pin GPIO_PIN_1
#define POWRADIO_GPIO_Port GPIOC
#define POWGPS_Pin GPIO_PIN_2
#define POWGPS_GPIO_Port GPIOC
#define POWBLU_Pin GPIO_PIN_3
#define POWBLU_GPIO_Port GPIOC
#define BatV_Pin GPIO_PIN_1
#define BatV_GPIO_Port GPIOA
#define STM32S_Pin GPIO_PIN_6
#define STM32S_GPIO_Port GPIOA
#define DO1_Pin GPIO_PIN_7
#define DO1_GPIO_Port GPIOA
#define DO2_Pin GPIO_PIN_4
#define DO2_GPIO_Port GPIOC
#define GPSPPS_Pin GPIO_PIN_5
#define GPSPPS_GPIO_Port GPIOC
#define GPSPPS_EXTI_IRQn EXTI9_5_IRQn
#define BLUSI_Pin GPIO_PIN_0
#define BLUSI_GPIO_Port GPIOB
#define T4GSI_Pin GPIO_PIN_1
#define T4GSI_GPIO_Port GPIOB
#define T4GSI_EXTI_IRQn EXTI1_IRQn
#define DO3_Pin GPIO_PIN_12
#define DO3_GPIO_Port GPIOB
#define DI1_Pin GPIO_PIN_13
#define DI1_GPIO_Port GPIOB
#define DI2_Pin GPIO_PIN_14
#define DI2_GPIO_Port GPIOB
#define DI3_Pin GPIO_PIN_15
#define DI3_GPIO_Port GPIOB
#define LASER_Pin GPIO_PIN_8
#define LASER_GPIO_Port GPIOC
#define RADIOS_Pin GPIO_PIN_9
#define RADIOS_GPIO_Port GPIOC
#define T4GS_Pin GPIO_PIN_8
#define T4GS_GPIO_Port GPIOA
#define T4GCS_Pin GPIO_PIN_11
#define T4GCS_GPIO_Port GPIOA
#define BLUS_Pin GPIO_PIN_12
#define BLUS_GPIO_Port GPIOA
#define GPSS_Pin GPIO_PIN_15
#define GPSS_GPIO_Port GPIOA
#define T4GCI_Pin GPIO_PIN_3
#define T4GCI_GPIO_Port GPIOB
#define T4GRES_Pin GPIO_PIN_4
#define T4GRES_GPIO_Port GPIOB
#define RADIOM0_Pin GPIO_PIN_5
#define RADIOM0_GPIO_Port GPIOB
#define GPSRES_Pin GPIO_PIN_6
#define GPSRES_GPIO_Port GPIOB
#define RADIOM1_Pin GPIO_PIN_7
#define RADIOM1_GPIO_Port GPIOB
#define RADIOSI_Pin GPIO_PIN_8
#define RADIOSI_GPIO_Port GPIOB
#define BLURES_Pin GPIO_PIN_9
#define BLURES_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern IWDG_HandleTypeDef hiwdg;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
