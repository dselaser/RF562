/*===========================================================================*\
|=============================================================================|
|                                                                             |
| @file   cnnx_util.c                                                         |
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


#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "cnnx_util.h"


//   define your monitoring console USART port here
#
static UART_HandleTypeDef   *pConsole = &CNNX_CONSOLE_PORT ;

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  HAL_UART_Transmit(pConsole, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void
console_flush(void)
{
	HAL_UART_Transmit(pConsole, (uint8_t*)NULL, 0, 0xFFFF);
}



