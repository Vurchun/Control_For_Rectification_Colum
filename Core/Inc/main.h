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
#define Extra_Sensor_Pin GPIO_PIN_13
#define Extra_Sensor_GPIO_Port GPIOC
#define Control_TEH_Pin GPIO_PIN_14
#define Control_TEH_GPIO_Port GPIOC
#define Exstra_Power_Off_Pin GPIO_PIN_15
#define Exstra_Power_Off_GPIO_Port GPIOC
#define Cooler_Motor_Pin GPIO_PIN_0
#define Cooler_Motor_GPIO_Port GPIOA
#define Cooler_Relay_Pin GPIO_PIN_1
#define Cooler_Relay_GPIO_Port GPIOA
#define Multy_Relay_4_Pin GPIO_PIN_12
#define Multy_Relay_4_GPIO_Port GPIOB
#define Multy_Relay_3_Pin GPIO_PIN_13
#define Multy_Relay_3_GPIO_Port GPIOB
#define Multy_Relay_1_Pin GPIO_PIN_14
#define Multy_Relay_1_GPIO_Port GPIOB
#define Multy_Relay_0_Pin GPIO_PIN_15
#define Multy_Relay_0_GPIO_Port GPIOB
#define Frequincy_Counter_Pin GPIO_PIN_11
#define Frequincy_Counter_GPIO_Port GPIOA
#define Frequincy_Control_Pin GPIO_PIN_12
#define Frequincy_Control_GPIO_Port GPIOA
#define Enc_CK_Pin GPIO_PIN_15
#define Enc_CK_GPIO_Port GPIOA
#define Enc_DT_Pin GPIO_PIN_3
#define Enc_DT_GPIO_Port GPIOB
#define Enc_SW_Pin GPIO_PIN_4
#define Enc_SW_GPIO_Port GPIOB
#define DS18b20_Pin GPIO_PIN_5
#define DS18b20_GPIO_Port GPIOB
#define BME280_SCL_Pin GPIO_PIN_6
#define BME280_SCL_GPIO_Port GPIOB
#define BME280_SDA_Pin GPIO_PIN_7
#define BME280_SDA_GPIO_Port GPIOB
#define Selection_Motor_Pin GPIO_PIN_8
#define Selection_Motor_GPIO_Port GPIOB
#define Selection_Relay_Pin GPIO_PIN_9
#define Selection_Relay_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
