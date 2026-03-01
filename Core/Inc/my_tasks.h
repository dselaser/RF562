/*
 * my_task.h
 *
 *  Created on: Dec 13, 2025
 *      Author: hl3xs
 */


#ifndef MY_TASKS_H
#define MY_TASKS_H
#include "cmsis_os2.h" // osThreadId_t 타입 사용을 위해 포함
#define UART_TX_CPLT_NOTIFY_BIT (1U << 0)

#pragma once
#include "FreeRTOS.h"
#include "task.h"
extern TaskHandle_t gUart2TxTaskHandle;
/* FreeRTOS Task entry prototypes */
void ADS8325_AcqTask(void *argument);

void LEDTask(void *argument);
void SysMonTask(void *argument);
void Loop1_TIM6_ISR_Handler(void);

#endif /* MY_TASKS_H */
