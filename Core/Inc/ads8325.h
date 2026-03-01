#ifndef ADS8325_H
#define ADS8325_H

#include "main.h"

#include <stdint.h>
#include <stddef.h>

// 1) 핸들 타입 정의 (ads8325.c 에서 쓰는 이름에 맞춤)
typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
} ADS8325_Handle_t;

// 2) 전역 핸들 (ads8325.c 에서 실제로 정의할 것)
extern ADS8325_Handle_t g_ads8325;

// 3) 함수 프로토타입들 (에러 로그에 나온 ads8325.c 시그니처에 맞춤)

uint16_t ADS8325_Read(const ADS8325_Handle_t *hadc);

HAL_StatusTypeDef ADS8325_ReadN(const ADS8325_Handle_t *hadc,
                                uint16_t *dst,
                                size_t    n);

HAL_StatusTypeDef ADS8325_ReadN_Averaged(const ADS8325_Handle_t *hadc,
                                         uint16_t *dst,
                                         size_t    n,
                                         uint8_t   avg_count);

HAL_StatusTypeDef ADS8325_Init(ADS8325_Handle_t *hadc,
                               SPI_HandleTypeDef *hspi,
                               GPIO_TypeDef *cs_port,
                               uint16_t cs_pin);


#endif /* ADS8325_H */
