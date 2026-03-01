/* uart_cmd.h */

#ifndef UART_CMD_H
#define UART_CMD_H

#include "stm32h5xx_hal.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 전역 UART1 수신 1바이트 버퍼 (IT 재예약용) */
extern volatile uint8_t g_uart1_rx_byte;

void UART_Cmd_Init(void);                 // 큐 생성, 수신 시작, 태스크 생성
void UART_CmdTask(void *arg);             // 수신 바이트 처리 태스크
void UART_Cmd_OnRxByteFromISR(uint8_t ch);// ISR에서 1바이트 전달용


void UART_Cmd_PrimeTaskStatsBaseline(void);
void UART_Cmd_PrintTaskStatsNow(void);



size_t UART_Cmd_GetRxQueueFree(void);
size_t UART_Cmd_GetRxQueueLength(void);


#ifdef __cplusplus
}
#endif
#endif

