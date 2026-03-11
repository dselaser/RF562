/*
 * adc_spi1.c
 *
 *  Created on: Nov 12, 2025
 *      Author: hl3xs
 */

#include "ads8325.h"

ADS8325_Handle_t g_ads8325;

HAL_StatusTypeDef ADS8325_Init(ADS8325_Handle_t *hadc,
                               SPI_HandleTypeDef *hspi,
                               GPIO_TypeDef *cs_port,
                               uint16_t cs_pin)
{
    if (hadc == NULL || hspi == NULL || cs_port == NULL)
        return HAL_ERROR;

    hadc->hspi    = hspi;
    hadc->cs_port = cs_port;
    hadc->cs_pin  = cs_pin;

    // 필요하면 여기에서 CS 핀을 HIGH로 올려두기
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    return HAL_OK;
}

uint16_t ADS8325_Read(const ADS8325_Handle_t *hadc)
{
    uint8_t tx[3] = {0xFF, 0xFF, 0xFF};
    uint8_t rx[3] = {0, 0, 0};
    uint32_t val;

    /* CS Low → 샘플링 시작 */
    HAL_GPIO_WritePin(hadc->cs_port, hadc->cs_pin, GPIO_PIN_RESET);

    /* 짧은 setup 여유(수 ns면 충분, NOP 몇 개면 OK) */
    __NOP(); __NOP(); __NOP(); __NOP();

    if (HAL_SPI_TransmitReceive(hadc->hspi, tx, rx, 3, 5) != HAL_OK) {
        HAL_GPIO_WritePin(hadc->cs_port, hadc->cs_pin, GPIO_PIN_SET);
        return 0;
    }

    /* 변환/시프트 종료 */
    HAL_GPIO_WritePin(hadc->cs_port, hadc->cs_pin, GPIO_PIN_SET);

    val = ((uint32_t)(rx[0] & 0x7FU) << 9) | ((uint32_t)rx[1] << 1) | ((uint32_t)rx[2] >> 7);

    return (uint16_t)(val & 0xFFFFU);
}

HAL_StatusTypeDef ADS8325_ReadN(const ADS8325_Handle_t *hadc, uint16_t *dst, size_t n)
{
    if (!dst) return HAL_ERROR;

    for (size_t i = 0; i < n; ++i) {
        dst[i] = ADS8325_Read(hadc);
    }
    return HAL_OK;
}

// N개의 "평균 샘플"을 만드는 함수
// avgN = 한 샘플당 몇 번 읽어서 평균낼지 (예: 10)
HAL_StatusTypeDef ADS8325_ReadN_Averaged(const ADS8325_Handle_t *hadc,
                                         uint16_t *dst,
                                         size_t n,
                                         uint8_t avgN)
{
    if (!dst || avgN == 0) return HAL_ERROR;

    for (size_t i = 0; i < n; ++i)
    {
        uint32_t sum = 0;

        for (uint8_t k = 0; k < avgN; ++k) {
            sum += ADS8325_Read(hadc);  // 16비트 값, 10번 합해도 32비트면 충분
        }

        dst[i] = (uint16_t)(sum / avgN);
    }

    return HAL_OK;
}
