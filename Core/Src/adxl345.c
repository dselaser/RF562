/*
 * adxl345.c
 *
 *  Created on: Dec 12, 2025
 *      Author: hl3xs
 */


#include "adxl345.h"
#include "i2c.h"   // hi2c2
#include "gpio.h"


#include "usart.h"
#include "cmsis_os2.h"
#include <stdio.h>



extern I2C_HandleTypeDef hi2c2;

// ADXL345 7-bit 주소 = 0x53, HAL은 (addr<<1) 사용
#define ADXL345_I2C_ADDR   (0x53 << 1)

// 레지스터 주소
#define ADXL345_REG_DEVID       0x00
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_BW_RATE     0x2C
#define ADXL345_REG_DATAX0      0x32  // X0~Z1까지 6바이트

// 간단한 레지스터 쓰기 함수
static HAL_StatusTypeDef adxl345_write_reg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c2,
                             ADXL345_I2C_ADDR,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1,
                             10);
}

// 간단한 레지스터 읽기 함수
static HAL_StatusTypeDef adxl345_read_reg(uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(&hi2c2,
                            ADXL345_I2C_ADDR,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1,
                            10);
}

void ADXL345_Init_I2C2(void)
{
    uint8_t dev_id = 0;

    // 1) Device ID 확인 (선택 사항: 0xE5가 나오는지)
    if (adxl345_read_reg(ADXL345_REG_DEVID, &dev_id) == HAL_OK)
    {
        // 필요하면 dev_id 검사해서 디버그 출력 가능
        // ex) if (dev_id != 0xE5) 에러 처리
    }

    // 2) 측정모드로 설정 (POWER_CTL: Measure bit = 1)
    //    POWER_CTL (0x2D): 0x08 = Measure mode
    adxl345_write_reg(ADXL345_REG_POWER_CTL, 0x08);

    // 3) 데이터 포맷 설정 (풀 해상도, ±2g)
    //    DATA_FORMAT (0x31): FULL_RES=1 (bit3), Range=±2g(00)
    //    -> 0x08
    adxl345_write_reg(ADXL345_REG_DATA_FORMAT, 0x08);

    // 4) 출력 데이터 속도 설정 (예: 100 Hz)
    //    BW_RATE (0x2C): 0x0A = 100 Hz
    adxl345_write_reg(ADXL345_REG_BW_RATE, 0x0A);
}

HAL_StatusTypeDef ADXL345_ReadRaw_I2C2(ADXL345_Raw_t *out)
{
    uint8_t buf[6];

    HAL_StatusTypeDef ret =
        HAL_I2C_Mem_Read(&hi2c2,
                         ADXL345_I2C_ADDR,
                         ADXL345_REG_DATAX0,
                         I2C_MEMADD_SIZE_8BIT,
                         buf,
                         6,
                         10);

    if (ret != HAL_OK)
    {
        return ret;
    }

    // 데이터는 little-endian (X0, X1, Y0, Y1, Z0, Z1)
    out->x = (int16_t)((buf[1] << 8) | buf[0]);
    out->y = (int16_t)((buf[3] << 8) | buf[2]);
    out->z = (int16_t)((buf[5] << 8) | buf[4]);

    return HAL_OK;
}


void ADXL345_UART2_Task(void *argument)
{
    (void)argument;

    ADXL345_Raw_t acc;
    char buf[64];
    int  len;

    // ADXL345 초기화
    ADXL345_Init_I2C2();

    // 센서 안정화 대기
    osDelay(50);

    for (;;)
    {
        if (ADXL345_ReadRaw_I2C2(&acc) == HAL_OK)
        {
            // raw 값 출력 (원하면 g 단위로 변환 가능)
            // ADXL345 full-res, ±2g 일 때 약 3.9 mg/LSB
            // float ax_g = acc.x * 0.0039f / 1000.0f; 등으로 계산 가능.
            len = snprintf(buf,
                           sizeof(buf),
                           "AX=%d AY=%d AZ=%d\r\n",
                           acc.x,
                           acc.y,
                           acc.z);
        }
        else
        {
            len = snprintf(buf, sizeof(buf), "ADXL345 ERR\r\n");
        }

        // ---- RS-485 송신 (UART2) ----
        // DE Enable (송신 모드)
     //   HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_SET);

     //   HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 50);

        // 송신 완료 후 약간의 여유를 주고 DE 비활성(수신모드)
        // (보드 상황에 따라 delay는 줄이거나 제거 가능)
        // osDelay(1);
     //   HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET);
        // ------------------------------

        // 샘플링/전송 주기 (예: 50ms → 20Hz)
        osDelay(50);
    }
}
