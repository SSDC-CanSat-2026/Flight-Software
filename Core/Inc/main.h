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
#include "stm32g4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STAT_BKUP_Pin GPIO_PIN_13
#define STAT_BKUP_GPIO_Port GPIOC
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define CHG_STAT2_Pin GPIO_PIN_0
#define CHG_STAT2_GPIO_Port GPIOF
#define CHG_STAT1_Pin GPIO_PIN_1
#define CHG_STAT1_GPIO_Port GPIOF
#define ENC0_A_Pin GPIO_PIN_0
#define ENC0_A_GPIO_Port GPIOC
#define ENC0_B_Pin GPIO_PIN_1
#define ENC0_B_GPIO_Port GPIOC
#define EN_5V_Pin GPIO_PIN_2
#define EN_5V_GPIO_Port GPIOC
#define VUSB_Pin GPIO_PIN_3
#define VUSB_GPIO_Port GPIOC
#define ENC1_Z_Pin GPIO_PIN_0
#define ENC1_Z_GPIO_Port GPIOA
#define GPS_1PPS_Pin GPIO_PIN_1
#define GPS_1PPS_GPIO_Port GPIOA
#define CLK_32k_Pin GPIO_PIN_2
#define CLK_32k_GPIO_Port GPIOA
#define SERVO_PWM4_Pin GPIO_PIN_3
#define SERVO_PWM4_GPIO_Port GPIOA
#define CAM0_CTRL_Pin GPIO_PIN_5
#define CAM0_CTRL_GPIO_Port GPIOA
#define SERVO_PWM0_Pin GPIO_PIN_6
#define SERVO_PWM0_GPIO_Port GPIOA
#define SERVO_PWM1_Pin GPIO_PIN_7
#define SERVO_PWM1_GPIO_Port GPIOA
#define ENC0_Z_Pin GPIO_PIN_4
#define ENC0_Z_GPIO_Port GPIOC
#define CAM1_CTRL_Pin GPIO_PIN_5
#define CAM1_CTRL_GPIO_Port GPIOC
#define SERVO_PWM2_Pin GPIO_PIN_0
#define SERVO_PWM2_GPIO_Port GPIOB
#define SERVO_PWM3_Pin GPIO_PIN_1
#define SERVO_PWM3_GPIO_Port GPIOB
#define IMU_nCS_Pin GPIO_PIN_2
#define IMU_nCS_GPIO_Port GPIOB
#define MAGEXT_nCS_Pin GPIO_PIN_10
#define MAGEXT_nCS_GPIO_Port GPIOB
#define MAG_nCS_Pin GPIO_PIN_11
#define MAG_nCS_GPIO_Port GPIOB
#define BMP_nCS_Pin GPIO_PIN_12
#define BMP_nCS_GPIO_Port GPIOB
#define ENC1_A_Pin GPIO_PIN_6
#define ENC1_A_GPIO_Port GPIOC
#define ENC1_B_Pin GPIO_PIN_7
#define ENC1_B_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define XBEE_RST_Pin GPIO_PIN_15
#define XBEE_RST_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_12
#define GPS_RX_GPIO_Port GPIOC
#define GPS_TX_Pin GPIO_PIN_2
#define GPS_TX_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define GPS_RST_Pin GPIO_PIN_5
#define GPS_RST_GPIO_Port GPIOB
#define USR_LED_Pin GPIO_PIN_7
#define USR_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
