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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f7xx_hal.h"

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
#define qtr_ch16_Pin GPIO_PIN_3
#define qtr_ch16_GPIO_Port GPIOF
#define qtr_ch15_Pin GPIO_PIN_4
#define qtr_ch15_GPIO_Port GPIOF
#define qtr_ch14_Pin GPIO_PIN_5
#define qtr_ch14_GPIO_Port GPIOF
#define qtr_ch13_Pin GPIO_PIN_6
#define qtr_ch13_GPIO_Port GPIOF
#define qtr_ch12_Pin GPIO_PIN_7
#define qtr_ch12_GPIO_Port GPIOF
#define qtr_ch11_Pin GPIO_PIN_8
#define qtr_ch11_GPIO_Port GPIOF
#define qtr_ch10_Pin GPIO_PIN_9
#define qtr_ch10_GPIO_Port GPIOF
#define qtr_ch9_Pin GPIO_PIN_10
#define qtr_ch9_GPIO_Port GPIOF
#define qtr_ctrl_odd_Pin GPIO_PIN_0
#define qtr_ctrl_odd_GPIO_Port GPIOH
#define qtr_ctrl_even_Pin GPIO_PIN_1
#define qtr_ctrl_even_GPIO_Port GPIOH
#define qtr_ch8_Pin GPIO_PIN_0
#define qtr_ch8_GPIO_Port GPIOC
#define qtr_ch7_Pin GPIO_PIN_1
#define qtr_ch7_GPIO_Port GPIOC
#define qtr_ch6_Pin GPIO_PIN_2
#define qtr_ch6_GPIO_Port GPIOC
#define qtr_ch5_Pin GPIO_PIN_3
#define qtr_ch5_GPIO_Port GPIOC
#define lid1_txd_Pin GPIO_PIN_0
#define lid1_txd_GPIO_Port GPIOA
#define lid1_rxd_Pin GPIO_PIN_1
#define lid1_rxd_GPIO_Port GPIOA
#define tele_txd_Pin GPIO_PIN_2
#define tele_txd_GPIO_Port GPIOA
#define tele_rxd_Pin GPIO_PIN_3
#define tele_rxd_GPIO_Port GPIOA
#define Vbatt_Pin GPIO_PIN_4
#define Vbatt_GPIO_Port GPIOA
#define SpeedIn_Pin GPIO_PIN_5
#define SpeedIn_GPIO_Port GPIOA
#define DirIn_Pin GPIO_PIN_6
#define DirIn_GPIO_Port GPIOA
#define Vesc_Pin GPIO_PIN_7
#define Vesc_GPIO_Port GPIOA
#define qtr_ch4_Pin GPIO_PIN_4
#define qtr_ch4_GPIO_Port GPIOC
#define qtr_ch3_Pin GPIO_PIN_5
#define qtr_ch3_GPIO_Port GPIOC
#define qtr_ch2_Pin GPIO_PIN_0
#define qtr_ch2_GPIO_Port GPIOB
#define qtr_ch1_Pin GPIO_PIN_1
#define qtr_ch1_GPIO_Port GPIOB
#define lid1_pwr_en_Pin GPIO_PIN_2
#define lid1_pwr_en_GPIO_Port GPIOB
#define lid6_pwr_en_Pin GPIO_PIN_11
#define lid6_pwr_en_GPIO_Port GPIOF
#define lid2_rxd_Pin GPIO_PIN_7
#define lid2_rxd_GPIO_Port GPIOE
#define lid2_txd_Pin GPIO_PIN_8
#define lid2_txd_GPIO_Port GPIOE
#define SpeedOut_Pin GPIO_PIN_9
#define SpeedOut_GPIO_Port GPIOE
#define DirOut_Pin GPIO_PIN_11
#define DirOut_GPIO_Port GPIOE
#define SpareOut_Pin GPIO_PIN_13
#define SpareOut_GPIO_Port GPIOE
#define lid3_txd_Pin GPIO_PIN_10
#define lid3_txd_GPIO_Port GPIOB
#define lid3_rxd_Pin GPIO_PIN_11
#define lid3_rxd_GPIO_Port GPIOB
#define lid2_pwr_en_Pin GPIO_PIN_12
#define lid2_pwr_en_GPIO_Port GPIOB
#define lid3_pwr_en_Pin GPIO_PIN_13
#define lid3_pwr_en_GPIO_Port GPIOB
#define SpeedSensor_Pin GPIO_PIN_14
#define SpeedSensor_GPIO_Port GPIOB
#define SpareIn_Pin GPIO_PIN_12
#define SpareIn_GPIO_Port GPIOD
#define Spare7_Pin GPIO_PIN_15
#define Spare7_GPIO_Port GPIOD
#define Spare6_Pin GPIO_PIN_2
#define Spare6_GPIO_Port GPIOG
#define Spare5_Pin GPIO_PIN_3
#define Spare5_GPIO_Port GPIOG
#define Spare4_Pin GPIO_PIN_4
#define Spare4_GPIO_Port GPIOG
#define Spare3_Pin GPIO_PIN_5
#define Spare3_GPIO_Port GPIOG
#define Spare2_Pin GPIO_PIN_6
#define Spare2_GPIO_Port GPIOG
#define Spare1_Pin GPIO_PIN_7
#define Spare1_GPIO_Port GPIOG
#define Spare0_Pin GPIO_PIN_8
#define Spare0_GPIO_Port GPIOG
#define lid4_txd_Pin GPIO_PIN_9
#define lid4_txd_GPIO_Port GPIOA
#define lid4_rxd_Pin GPIO_PIN_10
#define lid4_rxd_GPIO_Port GPIOA
#define lid4_pwr_en_Pin GPIO_PIN_15
#define lid4_pwr_en_GPIO_Port GPIOA
#define lid5_pwr_en_Pin GPIO_PIN_10
#define lid5_pwr_en_GPIO_Port GPIOC
#define lid5_txd_Pin GPIO_PIN_12
#define lid5_txd_GPIO_Port GPIOC
#define lid5_rxd_Pin GPIO_PIN_2
#define lid5_rxd_GPIO_Port GPIOD
#define boutonExt1_Pin GPIO_PIN_4
#define boutonExt1_GPIO_Port GPIOD
#define boutonExt2_Pin GPIO_PIN_5
#define boutonExt2_GPIO_Port GPIOD
#define led3_Pin GPIO_PIN_6
#define led3_GPIO_Port GPIOD
#define led2_Pin GPIO_PIN_7
#define led2_GPIO_Port GPIOD
#define led1_Pin GPIO_PIN_9
#define led1_GPIO_Port GPIOG
#define bouton3_Pin GPIO_PIN_10
#define bouton3_GPIO_Port GPIOG
#define bouton2_Pin GPIO_PIN_11
#define bouton2_GPIO_Port GPIOG
#define bouton1_Pin GPIO_PIN_12
#define bouton1_GPIO_Port GPIOG
#define cmdLeds_Pin GPIO_PIN_5
#define cmdLeds_GPIO_Port GPIOB
#define lid6_rxd_Pin GPIO_PIN_0
#define lid6_rxd_GPIO_Port GPIOE
#define lid6_txd_Pin GPIO_PIN_1
#define lid6_txd_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
