/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define Q0_0_Pin GPIO_PIN_13
#define Q0_0_GPIO_Port GPIOC
#define Q0_1_Pin GPIO_PIN_0
#define Q0_1_GPIO_Port GPIOA
#define MBUS_CTRL_Pin GPIO_PIN_1
#define MBUS_CTRL_GPIO_Port GPIOA
#define MBUS_TX_Pin GPIO_PIN_2
#define MBUS_TX_GPIO_Port GPIOA
#define MBUS_RX_Pin GPIO_PIN_3
#define MBUS_RX_GPIO_Port GPIOA
#define A3V3_Pin GPIO_PIN_0
#define A3V3_GPIO_Port GPIOB
#define A5V_Pin GPIO_PIN_1
#define A5V_GPIO_Port GPIOB
#define IA0_Pin GPIO_PIN_2
#define IA0_GPIO_Port GPIOB
#define IA1_Pin GPIO_PIN_10
#define IA1_GPIO_Port GPIOB
#define ETH_SCK_Pin GPIO_PIN_12
#define ETH_SCK_GPIO_Port GPIOB
#define FALLA_Pin GPIO_PIN_13
#define FALLA_GPIO_Port GPIOB
#define CNN_Pin GPIO_PIN_14
#define CNN_GPIO_Port GPIOB
#define ALIM_Pin GPIO_PIN_15
#define ALIM_GPIO_Port GPIOB
#define WF_EN_RST_Pin GPIO_PIN_8
#define WF_EN_RST_GPIO_Port GPIOA
#define WIFI_TX_Pin GPIO_PIN_9
#define WIFI_TX_GPIO_Port GPIOA
#define WIFI_RX_Pin GPIO_PIN_10
#define WIFI_RX_GPIO_Port GPIOA
#define DBG_TX_Pin GPIO_PIN_11
#define DBG_TX_GPIO_Port GPIOA
#define DBG_RX_Pin GPIO_PIN_12
#define DBG_RX_GPIO_Port GPIOA
#define DBG_PIN_Pin GPIO_PIN_15
#define DBG_PIN_GPIO_Port GPIOA
#define ETH_MISO_Pin GPIO_PIN_4
#define ETH_MISO_GPIO_Port GPIOB
#define ETH_MOSI_Pin GPIO_PIN_5
#define ETH_MOSI_GPIO_Port GPIOB
#define ETH_NSS_Pin GPIO_PIN_6
#define ETH_NSS_GPIO_Port GPIOB
#define ETH_RST_Pin GPIO_PIN_7
#define ETH_RST_GPIO_Port GPIOB
#define LR_RST_Pin GPIO_PIN_9
#define LR_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
