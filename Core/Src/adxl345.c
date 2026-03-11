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
#include <math.h>



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

        HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_SET);
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 50);
        HAL_GPIO_WritePin(USART2_DE_GPIO_Port, USART2_DE_Pin, GPIO_PIN_RESET);

        osDelay(50);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  LCD 자동 회전 (ADXL345 기반)
 *
 *  X축 = 니들 방향 (실측 확인 완료)
 *    - 니들 DOWN: Px ≈ +87°   (tilt_from_down ≈  3°)
 *    - 니들 UP  : Px ≈ -87°   (tilt_from_down ≈ 177°)
 *
 *  전환 기준: tilt_from_down = 90° → Px = 90° - 90° = 0°
 *    히스테리시스 ±10°:
 *      DOWN→UP 전환: Px < -10° (tilt > 100°)
 *      UP→DOWN 전환: Px > +10° (tilt <  80°)
 *
 *  g_screen_rotated: 0=정상(니들DOWN), 1=180°회전(니들UP)
 *    → lvglHandler에서 MADCTL 적용, touchpad_read에서 좌표 반전
 * ═══════════════════════════════════════════════════════════════════════════*/
volatile uint8_t g_screen_rotated = 0;

void ADXL345_UART1_Task(void *argument)
{
    (void)argument;

    ADXL345_Raw_t acc;

    /* ADXL345 초기화 */
    ADXL345_Init_I2C2();
    osDelay(100);   /* 센서 안정화 */

    /* 히스테리시스 임계값 (Px 도 단위) */
    const float THRESH_TO_UP   = -10.0f;  /* Px < -10° → 화면 180° 회전 */
    const float THRESH_TO_DOWN =  10.0f;  /* Px > +10° → 화면 정상       */

    for (;;)
    {
        if (ADXL345_ReadRaw_I2C2(&acc) == HAL_OK)
        {
            /* ±2g full-res: 256 LSB/g → g 단위 변환 */
            float ax = acc.x / 256.0f;
            float ay = acc.y / 256.0f;
            float az = acc.z / 256.0f;

            /* Px: X축(니들 축) pitch 각도
             *   니들DOWN → +87°,  니들UP → -87°,  수평 → 0° */
            float px = atan2f(ax, sqrtf(ay*ay + az*az))
                        * (180.0f / 3.14159265f);

            /* ── 히스테리시스 화면 회전 판정 ───────────────────── */
            if (!g_screen_rotated && px < THRESH_TO_UP) {
                g_screen_rotated = 1;   /* 니들 UP → 화면 180° 회전 */
            }
            else if (g_screen_rotated && px > THRESH_TO_DOWN) {
                g_screen_rotated = 0;   /* 니들 DOWN → 화면 정상 */
            }

#if 0   /* ── UART1 디버그 출력 (필요 시 1로 변경) ──────────── */
            {
                extern UART_HandleTypeDef huart1;
                char buf[128];
                int  len = snprintf(buf, sizeof(buf),
                    "Px=%4d R=%d  X=%4d Y=%4d Z=%4d\r\n",
                    (int)px, g_screen_rotated,
                    acc.x, acc.y, acc.z);
                HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
            }
#endif
        }

        osDelay(200);   /* 5 Hz 샘플링 */
    }
}
