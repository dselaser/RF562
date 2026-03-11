/*===========================================================================*\
|=============================================================================|
|                                                                             |
| @file   cnnx_util.h                                                         |
| @author connexthings@naver.com                                              |
| @brief  general utilities for STM32 series                                  |
| @version   0.8                                                              |
|                                                                             |
|=============================================================================|
|                                                                             |
| @section  LICENSE                                                           |
|                                                                             |
|   Copyright (c) 2024, connexthings@naver.com                                |
|   All commercial right reserved under the International Copyright Law       |
|                                                                             |
|  This is UNPUBLISHED PROPRIETARY SOURCE CODE of conneXthings.com;           |
|  the contents of this file may not be disclosed to third parties, copied,   |
|  duplicated, deleted or saled in any form, in whole, or in part without     |
|  the prior written permission of the international copyright holder.        |
|                                                                             |
|=============================================================================|
|                                                                             |
|  Usage                                                                      |
|  i) include STM32 hal header like "stm32f4xx_hal.h" before this header,     |
|     or it will spout errors.                                                |
|  ii) define the monitering console USART port below                         |
|  iii) enable and initialize the monitering console USART port               |
|       before using printf()                                                 |
|                                                                             |
|=============================================================================|
|                                                                             |
|  Revision History                                                           |
|                                                                             |
|  v0.8  : small printf() via USART                                           |
|                                                                             |
\*===========================================================================*/

#ifndef   _CNNX_UTILS_
#define   _CNNX_UTILS_

#include <stdio.h>
#include "main.h"


//   define your monitoring console USART port here
#ifndef    CNNX_CONSOLE_PORT
   extern UART_HandleTypeDef      huart1 ;
   #define    CNNX_CONSOLE_PORT   huart1
#endif


void console_flush(void) ;


// #define __CNNX_DEBUG
#undef __CNNX_BEDUG

#ifdef __CNNX_DEBUG
	#define CNNX_Debug(__info,...) printf(" >>> Debug : " __info,##__VA_ARGS__)
#else
	#define CNNX_Debug(__info,...)
#endif


#endif   //   __CNNX_UTILS__


