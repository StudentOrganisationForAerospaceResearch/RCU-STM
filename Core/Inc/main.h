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
#include "stm32l4xx_hal.h"

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
#define CONT_CK0_Pin GPIO_PIN_13
#define CONT_CK0_GPIO_Port GPIOC
#define CONT_CK1_Pin GPIO_PIN_14
#define CONT_CK1_GPIO_Port GPIOC
#define NOS2_LC_CLK_Pin GPIO_PIN_0
#define NOS2_LC_CLK_GPIO_Port GPIOC
#define NOS2_LC_DATA_Pin GPIO_PIN_1
#define NOS2_LC_DATA_GPIO_Port GPIOC
#define SOL7_Pin GPIO_PIN_0
#define SOL7_GPIO_Port GPIOA
#define SOL6_Pin GPIO_PIN_1
#define SOL6_GPIO_Port GPIOA
#define PBV1_Pin GPIO_PIN_5
#define PBV1_GPIO_Port GPIOC
#define PBV2_Pin GPIO_PIN_0
#define PBV2_GPIO_Port GPIOB
#define PBV3_Pin GPIO_PIN_1
#define PBV3_GPIO_Port GPIOB
#define SOL4_Pin GPIO_PIN_2
#define SOL4_GPIO_Port GPIOB
#define SOL5_Pin GPIO_PIN_12
#define SOL5_GPIO_Port GPIOB
#define TC1_NCS_Pin GPIO_PIN_6
#define TC1_NCS_GPIO_Port GPIOC
#define TC2_NCS_Pin GPIO_PIN_7
#define TC2_NCS_GPIO_Port GPIOC
#define SOL8A_Pin GPIO_PIN_8
#define SOL8A_GPIO_Port GPIOC
#define SOL8B_Pin GPIO_PIN_9
#define SOL8B_GPIO_Port GPIOC
#define PADBOX1_Pin GPIO_PIN_8
#define PADBOX1_GPIO_Port GPIOA
#define PADBOX2_Pin GPIO_PIN_11
#define PADBOX2_GPIO_Port GPIOA
#define SHEDAC_Pin GPIO_PIN_12
#define SHEDAC_GPIO_Port GPIOA
#define RELAY0_Pin GPIO_PIN_15
#define RELAY0_GPIO_Port GPIOA
#define RADIO_TX_Pin GPIO_PIN_10
#define RADIO_TX_GPIO_Port GPIOC
#define RADIO_RX_Pin GPIO_PIN_11
#define RADIO_RX_GPIO_Port GPIOC
#define RS422_TX_EN_Pin GPIO_PIN_12
#define RS422_TX_EN_GPIO_Port GPIOC
#define USART1_TX_EN_Pin GPIO_PIN_2
#define USART1_TX_EN_GPIO_Port GPIOD
#define RELAY1_Pin GPIO_PIN_4
#define RELAY1_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_5
#define LED_2_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_6
#define LED_1_GPIO_Port GPIOB
#define LED_0_Pin GPIO_PIN_7
#define LED_0_GPIO_Port GPIOB
#define NOS1_LC_CLK_Pin GPIO_PIN_8
#define NOS1_LC_CLK_GPIO_Port GPIOB
#define NOS1_LC_DATA_Pin GPIO_PIN_9
#define NOS1_LC_DATA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
