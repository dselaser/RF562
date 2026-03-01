/*
 * ads7041.c
 *
 *  Created on: Dec 12, 2025
 *      Author: hl3xs
 */


#include "ads7041.h"
#include "spi.h"   // hspi3 선언이 들어있는 헤더 (CubeMX 기준)
#include "gpio.h"  // CS 핀 정의용


#include "usart.h"
#include "cmsis_os2.h"
#include <stdio.h>

// !!! 여기는 실제 프로젝트에 맞게 수정하세요 !!!
#define ADS7041_CS_GPIO_Port   GPIOD
#define ADS7041_CS_Pin         GPIO_PIN_2

extern SPI_HandleTypeDef hspi3;

/**
 * @brief  ADS7041에서 12비트 값을 읽어온다 (SPI3 사용)
 * @note   ADS7041은 클럭 16 사이클 동안 데이터를 내보냄.
 *         상위비트에 정렬된 12bit 를 >> 4 해서 우측 정렬.
 */
uint16_t ADS7041_Read(void)
{
    uint8_t tx[2] = {0x00, 0x00};
    uint8_t rx[2] = {0, };

    // CS Low
    HAL_GPIO_WritePin(ADS7041_CS_GPIO_Port, ADS7041_CS_Pin, GPIO_PIN_RESET);

    // 16비트 프레임 전송/수신
    HAL_SPI_TransmitReceive(&hspi3, tx, rx, 2, 10);

    // CS High
    HAL_GPIO_WritePin(ADS7041_CS_GPIO_Port, ADS7041_CS_Pin, GPIO_PIN_SET);

    // 수신 데이터 합치기 (MSB first)
    uint16_t raw16 = ((uint16_t)rx[0] << 8) | rx[1];

    // ADS7041 12bit 데이터는 상위비트 정렬 -> 4비트 오른쪽 시프트
    uint16_t adc12 = raw16 >> 4;

    return adc12;
}




void ADS7041_UART2_Task(void *argument)
{
    (void)argument;

    char     buf[32];
    uint16_t adc_val;
    int      len;

    for (;;)
    {
        // 1) ADC 값 읽기
        adc_val = ADS7041_Read();

        // 2) 문자열로 변환 (예: "1234\r\n")
        len = snprintf(buf, sizeof(buf), " %u", (unsigned int)adc_val);

        // 3) UART2로 전송 (blocking, 간단 버전)
        // 예시
        HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_SET);   // TX enable
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 10);
        HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET); // RX enable


        // 4) 샘플링 주기 설정 (예: 1ms -> 1kS/s 정도)
        osDelay(10);
    }
}
