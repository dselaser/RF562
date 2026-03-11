/*===========================================================================*\
|=============================================================================|
|                                                                             |
| @file   cnnx_proj.h                                                         |
| @author connexthings@naver.com                                              |
| @brief  all the project dependent feature here                              |
| @version   0.8                                                              |
|                                                                             |
|=============================================================================|
|                                                                             |
| @section  LICENSE                                                           |
|                                                                             |
|   Copyright (c) 2025, connexthings@naver.com                                |
|   All commercial right reserved under the International Copyright Law       |
|                                                                             |
|  This is UNPUBLISHED PROPRIETARY SOURCE CODE of conneXthings.com;           |
|  the contents of this file may not be disclosed to third parties, copied,   |
|  duplicated, deleted or sailed in any form, in whole, or in part without    |
|  the prior written permission of the international copyright holder.        |
|                                                                             |
|=============================================================================|
|                                                                             |
|  Usage                                                                      |
|  i) make cnnx_util.c/h purely independent of projects,                      |
|     all the project dependent features supported in this header             |
|                                                                             |
|=============================================================================|
|                                                                             |
|  Revision History                                                           |
|                                                                             |
|  v0.8  : initial creation                                                   |
|                                                                             |
\*===========================================================================*/

#ifndef   _CNNX_PROJ_
#define   _CNNX_PROJ_

#include <stdio.h>
#include "main.h"
#include "cnnx_util.h"

// #undef __CNNX_FREERTOS_ON
#define __CNNX_FREERTOS_ON

#ifdef   __CNNX_FREERTOS_ON
#include "cmsis_os2.h"
#endif

//   define your console monitor USART port here
#ifndef    CNNX_CONSOLE_PORT
   extern UART_HandleTypeDef      huart1 ;
   #define    CNNX_CONSOLE_PORT   huart1
#endif

//   for the LCD SPI communication channel
//   Define handle of SPI port connected to the display
extern SPI_HandleTypeDef          hspi2 ;
#define HLCD_SPI                  hspi2

//   for LCD backlight PWM Timer
//   Now, Timer1 Ch.3 PWM
extern TIM_HandleTypeDef          htim1 ;
// extern TIM_HandleTypeDef htim1, htim2, htim3;
#define DEV_BL_TMR      htim1						//  LCD Back Light PWM timer handle
#define DEV_BL_TCH      TIM_CHANNEL_3				//  LCD Back Light PWM timer channel
#define DEV_BL_PIN		TIM1->CCR3 					//  GPIO PA10
// #define DEV_BL_PIN		TIM2->CCR1 				   //  GPIO PA0
// #define DEV_BL_PIN		TIM3->CCR2 				   //  GPIO PA7

// Define Other Setup Stuffs, lvgl, DMA, etc.

// #undef __CNNX_LVGL_ON
#define __CNNX_LVGL_ON
// #undef  __CNNX_LCD_DMA_ON
#define __CNNX_LCD_DMA_ON


#endif   //   __CNNX_PROJ__


