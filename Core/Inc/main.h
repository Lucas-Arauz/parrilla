/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define HC595_OE_Pin GPIO_PIN_0
#define HC595_OE_GPIO_Port GPIOA
#define HC595_LACLK_Pin GPIO_PIN_1
#define HC595_LACLK_GPIO_Port GPIOA
#define HC595_SHCLK_Pin GPIO_PIN_2
#define HC595_SHCLK_GPIO_Port GPIOA
#define HC595_RESET_Pin GPIO_PIN_3
#define HC595_RESET_GPIO_Port GPIOA
#define HC595_DATA_Pin GPIO_PIN_4
#define HC595_DATA_GPIO_Port GPIOA
#define DISP_UNIDAD_Pin GPIO_PIN_5
#define DISP_UNIDAD_GPIO_Port GPIOA
#define DISP_DECENA_Pin GPIO_PIN_6
#define DISP_DECENA_GPIO_Port GPIOA
#define DISP_CENTENA_Pin GPIO_PIN_7
#define DISP_CENTENA_GPIO_Port GPIOA
#define CALEFACTOR_Pin GPIO_PIN_0
#define CALEFACTOR_GPIO_Port GPIOB
#define LED_INDIC_Pin GPIO_PIN_1
#define LED_INDIC_GPIO_Port GPIOB
#define LED_PROCESO_Pin GPIO_PIN_10
#define LED_PROCESO_GPIO_Port GPIOB
#define LED_CALIENTE_Pin GPIO_PIN_11
#define LED_CALIENTE_GPIO_Port GPIOB
#define TOUCH_ST_Pin GPIO_PIN_12
#define TOUCH_ST_GPIO_Port GPIOB
#define TOUCH_ST_EXTI_IRQn EXTI15_10_IRQn
#define TOUCH_TIEMPOMENOS_Pin GPIO_PIN_13
#define TOUCH_TIEMPOMENOS_GPIO_Port GPIOB
#define TOUCH_TIEMPOMENOS_EXTI_IRQn EXTI15_10_IRQn
#define TOUCH_TIEMPOMAS_Pin GPIO_PIN_14
#define TOUCH_TIEMPOMAS_GPIO_Port GPIOB
#define TOUCH_TIEMPOMAS_EXTI_IRQn EXTI15_10_IRQn
#define TOUCH_TEMPMENOS_Pin GPIO_PIN_15
#define TOUCH_TEMPMENOS_GPIO_Port GPIOB
#define TOUCH_TEMPMENOS_EXTI_IRQn EXTI15_10_IRQn
#define TOUCH_TEMPMAS_Pin GPIO_PIN_8
#define TOUCH_TEMPMAS_GPIO_Port GPIOA
#define TOUCH_TEMPMAS_EXTI_IRQn EXTI9_5_IRQn
#define MAX6675_NSS_Pin GPIO_PIN_15
#define MAX6675_NSS_GPIO_Port GPIOA
#define MAX6675_SCK_Pin GPIO_PIN_3
#define MAX6675_SCK_GPIO_Port GPIOB
#define MAX6675_MISO_Pin GPIO_PIN_4
#define MAX6675_MISO_GPIO_Port GPIOB
#define MAX6675_MOSI_Pin GPIO_PIN_5
#define MAX6675_MOSI_GPIO_Port GPIOB
#define MAX6675_CS_Pin GPIO_PIN_6
#define MAX6675_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
