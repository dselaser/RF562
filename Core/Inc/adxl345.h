/*
 * adxl345.h
 *
 *  Created on: Dec 12, 2025
 *      Author: hl3xs
 */

#ifndef __ADXL345_H__
#define __ADXL345_H__

#include "main.h"

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} ADXL345_Raw_t;

// I2C2 기반 초기화 / 읽기
void ADXL345_Init_I2C2(void);
HAL_StatusTypeDef ADXL345_ReadRaw_I2C2(ADXL345_Raw_t *out);

// LCD 자동 회전 상태: 0=정상(니들DOWN), 1=180°회전(니들UP)
extern volatile uint8_t g_screen_rotated;

// FreeRTOS 태스크
void ADXL345_UART1_Task(void *argument);   /* 화면 자동 회전 + 디버그 (UART1) */

#endif /* __ADXL345_H__ */

