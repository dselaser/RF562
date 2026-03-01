
// ===============================
// File: tle9201.c
// ===============================
#include "tle9201.h"
#include "stm32h5xx_hal.h"

static tle9201_spi_mode_t s_mode = TLE9201_SPI_MODE0;

#include "spi.h"
#include "gpio.h"
#include "tle9201.h"
#include "usart.h"   // huart1 선언( extern )

// CSN 핀 매크로 (필요 시 헤더로 이동)
#define TLE92XX_CSN_GPIO   GPIOD
#define TLE92XX_CSN_PIN    GPIO_PIN_2

static inline void CSN_L(void){ HAL_GPIO_WritePin(TLE92XX_CSN_GPIO, TLE92XX_CSN_PIN, GPIO_PIN_RESET); }
static inline void CSN_H(void){ HAL_GPIO_WritePin(TLE92XX_CSN_GPIO, TLE92XX_CSN_PIN, GPIO_PIN_SET); }

#include "tim.h"
#define TLE9201_TIM_CLK_HZ  (50000000UL)

static void TLE9201_SetPWM_Frequency(uint32_t freq_hz)
{
    if (freq_hz < 1000U)    freq_hz = 1000U;
    if (freq_hz > 100000U)  freq_hz = 100000U;

    __HAL_TIM_DISABLE(&htim8);                 // 카운터 정지
    __HAL_TIM_SET_PRESCALER(&htim8, 0);        // PSC = 0
    uint32_t arr = (uint32_t)((TLE9201_TIM_CLK_HZ + (freq_hz/2U)) / freq_hz) - 1U;
    if (arr < 1U) arr = 1U;
    __HAL_TIM_SET_AUTORELOAD(&htim8, arr);

    // CCR 범위 보정
    uint32_t ccr = __HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL_1);
    if (ccr > arr) { __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, arr); }

    __HAL_TIM_SET_COUNTER(&htim8, 0);
    HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_UPDATE);  // ★ 여기로 교체
    __HAL_TIM_ENABLE(&htim8);                                // 카운터 재시작
}



// 24-bit 프레임 전송 (8비트 x3, CSN Low 유지)
static void Xfer24(const uint8_t tx[3], uint8_t rx[3]){
    CSN_L();
    HAL_SPI_TransmitReceive(&hspi6, (uint8_t*)tx, rx, 3, 10); // 3 bytes, 24 clocks
    CSN_H();
}

// 파이프라인 응답: 명령 보낸 '다음 프레임'에 응답이 옴
static uint32_t ReadWithPipeline(const uint8_t cmd[3]){
    uint8_t dump[3], resp[3];
    // 1) 명령 프레임
    Xfer24(cmd, dump);          // 이 수신값은 이전 프레임 응답(의미 없음)
    // 2) 더미 프레임으로 실제 응답 뽑기
    const uint8_t nop[3] = {0x00,0x00,0x00};
    Xfer24(nop, resp);          // 여기 resp가 방금 명령에 대한 응답
    return ((uint32_t)resp[0]<<16)|((uint32_t)resp[1]<<8)|resp[2];
}

// 명령 바이트(예시는 데이터시트/보드 설정에 맞게 조정)
#define CMD_RD_REV_H  0x9F  // 예시: 상위바이트
#define CMD_RD_REV_M  0x00
#define CMD_RD_REV_L  0x00

#define CMD_RD_DIAG_H 0xA5  // 예시: 상위바이트
#define CMD_RD_DIAG_M 0x00
#define CMD_RD_DIAG_L 0x00

uint32_t TLE92xx_ReadREV(void){
    const uint8_t cmd[3] = {CMD_RD_REV_H, CMD_RD_REV_M, CMD_RD_REV_L};
    return ReadWithPipeline(cmd);
}

uint32_t TLE92xx_ReadDIAG(void){
    const uint8_t cmd[3] = {CMD_RD_DIAG_H, CMD_RD_DIAG_M, CMD_RD_DIAG_L};
    return ReadWithPipeline(cmd);
}


void TLE9201_Enable(bool en){
  if(!en){
    /* Disable 시 잔류 진동 방지:
     * - PWM duty를 먼저 0으로 만든 뒤 DIS=1(Hi-Z)로 차단
     */
    TLE9201_SetPWM_duty(0.0f);
    HAL_GPIO_WritePin(TLE9201_DIS_GPIO_Port, TLE9201_DIS_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(TLE9201_DIS_GPIO_Port, TLE9201_DIS_Pin, GPIO_PIN_RESET);
  }
}

void TLE9201_SetDir(bool forward){
  HAL_GPIO_WritePin(TLE9201_DIR_GPIO_Port, TLE9201_DIR_Pin, forward?GPIO_PIN_SET:GPIO_PIN_RESET);
}

void TLE9201_SetPWM_duty(float duty01){
	if (duty01 < 0.0f) duty01 = 0.0f;
	if (duty01 > 1.0f) duty01 = 1.0f;
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim8);
	uint32_t ccr = (uint32_t)((((float)arr + 1.0f) * duty01) + 0.5f);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, ccr);
}

// Configure SPI mode at runtime (Mode0 first, fallback to Mode1)
static void TLE9201_SetSPIMode(tle9201_spi_mode_t m){
  s_mode = m;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW; // CPOL=0 for both Mode0/1
  hspi6.Init.CLKPhase    = (m==TLE9201_SPI_MODE0)? SPI_PHASE_1EDGE : SPI_PHASE_2EDGE; // CPHA
  HAL_SPI_DeInit(&hspi6);
  HAL_SPI_Init(&hspi6);
}

uint16_t TLE9201_SPI_TxRx(uint16_t tx){
  uint8_t txb[2] = { (uint8_t)(tx>>8), (uint8_t)(tx & 0xFF) };
  uint8_t rxb[2] = {0};
  CSN_L();
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi6, txb, rxb, 2, 5);
  CSN_H();
  if(st != HAL_OK){ return 0xFFFF; }
  return ( (uint16_t)rxb[0]<<8 ) | rxb[1];
}

uint16_t TLE9201_ReadRev(void){
  // After power-up, the first SPI response (to any command) returns RD_REV per datasheet.
  // Send NOP (0x0000) as a safe dummy command.
  return TLE9201_SPI_TxRx(0x0000);
}

uint16_t TLE9201_ReadDiag(void){
  // Example placeholder: many Infineon bridges use 0x0001/0x8xxx patterns. Keep generic.
  // Replace 0x0001 with actual RD_STATUS command if you map it from the datasheet.
  return TLE9201_SPI_TxRx(0x0001);
}

void TLE9201_ClearErrors(void){
  // Example placeholder write that would clear latches; set actual value per datasheet.
  // e.g., write to CTRL register with CLR bits set.
  (void)TLE9201_SPI_TxRx(0x8000);
}

void TLE9201_Init(tle9201_spi_mode_t preferred_mode){
  // Ensure CSN high, H-bridge disabled during bring-up

  CSN_H();
  TLE9201_Enable(false);

  // Start PWM at 0% duty
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  __HAL_TIM_MOE_ENABLE(&htim8); // 필요시: 고급타이머(1/8) 출력 개방
  TLE9201_SetPWM_Frequency(40000);
  TLE9201_SetPWM_duty(0.0f);

  // Try preferred SPI mode first
  TLE9201_SetSPIMode(preferred_mode);
  uint16_t rev = TLE9201_ReadRev();

  // Heuristic: if readback is 0x0000 or 0xFFFF repeatedly, try alternate CPHA
  if(rev == 0x0000 || rev == 0xFFFF){
    TLE9201_SetSPIMode((preferred_mode==TLE9201_SPI_MODE0)?TLE9201_SPI_MODE1:TLE9201_SPI_MODE0);
    rev = TLE9201_ReadRev();
  }

  // Optionally, clear latched faults once SPI is alive
  TLE9201_ClearErrors();
}
