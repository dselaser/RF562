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
#include "stm32h5xx_hal.h"

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
#define ALIVE_Pin GPIO_PIN_13
#define ALIVE_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_0
#define LCD_CS_GPIO_Port GPIOC
#define USART2_DE_Pin GPIO_PIN_1
#define USART2_DE_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define DAC_Pin GPIO_PIN_5
#define DAC_GPIO_Port GPIOA
#define SHUTDOWN_Pin GPIO_PIN_5
#define SHUTDOWN_GPIO_Port GPIOC
#define LED_PIN_ON_Pin GPIO_PIN_0
#define LED_PIN_ON_GPIO_Port GPIOB
#define ADC1_INP5_Pin GPIO_PIN_1
#define ADC1_INP5_GPIO_Port GPIOB
#define VCA_PWM_Pin GPIO_PIN_6
#define VCA_PWM_GPIO_Port GPIOC
#define VCA_DIS_Pin GPIO_PIN_7
#define VCA_DIS_GPIO_Port GPIOC
#define VCA_DIR_Pin GPIO_PIN_8
#define VCA_DIR_GPIO_Port GPIOC
#define INT_ADXL_Pin GPIO_PIN_9
#define INT_ADXL_GPIO_Port GPIOC
#define HP_SW_Pin GPIO_PIN_8
#define HP_SW_GPIO_Port GPIOA
#define HP_WS2812_Pin GPIO_PIN_9
#define HP_WS2812_GPIO_Port GPIOA
#define LCD_RST_Pin GPIO_PIN_11
#define LCD_RST_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_12
#define LCD_DC_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define SPI6_CS_Pin GPIO_PIN_2
#define SPI6_CS_GPIO_Port GPIOD
#define I2C1_TP_SCL_Pin GPIO_PIN_6
#define I2C1_TP_SCL_GPIO_Port GPIOB
#define I2C1_TP_SDA_Pin GPIO_PIN_7
#define I2C1_TP_SDA_GPIO_Port GPIOB
#define TP_RST_Pin GPIO_PIN_8
#define TP_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void CNNX_I2C1_Init(void) ;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
