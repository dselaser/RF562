/*
 * speaker.c
 *
 *  Created on: Mar 2026
 *      Author: hl3xs
 *
 *  TPA2005D1 + DAC1_OUT2(PA5) 스피커 사운드 출력
 *  - 32-sample 사인파 LUT → DAC 소프트웨어 출력
 *  - DWT 사이클 카운터로 μs 정밀 타이밍
 *  - FreeRTOS 메시지 큐로 비동기 사운드 요청
 */

#include "speaker.h"
#include "dac.h"
#include "main.h"
#include "cmsis_os2.h"
#include "stm32h5xx.h"
#include "core_cm33.h"

extern DAC_HandleTypeDef hdac1;

/* ═══════════════════════════════════════════════════════════════════════════
 *  32-sample 사인파 LUT (12-bit, 0–4095, 중심 2048)
 * ═══════════════════════════════════════════════════════════════════════════*/
static const uint16_t sine32[32] = {
    2048, 2447, 2831, 3185, 3495, 3750, 3939, 4056,
    4095, 4056, 3939, 3750, 3495, 3185, 2831, 2447,
    2048, 1648, 1264,  910,  600,  345,  156,   39,
       0,   39,  156,  345,  600,  910, 1264, 1648
};

/* ── 사운드 요청 큐 ────────────────────────────────────────── */
static osMessageQueueId_t s_sndQueue;

/* ═══════════════════════════════════════════════════════════════════════════
 *  DWT 기반 μs 딜레이 (busy-wait)
 *  DWT->CYCCNT는 vConfigureTimerForRunTimeStats()에서 이미 활성화됨
 * ═══════════════════════════════════════════════════════════════════════════*/
static inline void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000UL);
    while ((DWT->CYCCNT - start) < ticks);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  앰프 ON/OFF  (SHUTDOWN: HIGH=ON, LOW=OFF)
 * ═══════════════════════════════════════════════════════════════════════════*/
static void amp_on(void)
{
    HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_SET);
}

static void amp_off(void)
{
    HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_RESET);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  tone()  –  지정 주파수/시간 동안 사인파 출력
 *
 *  freq_hz : 톤 주파수 (Hz), 200–4000 권장
 *  dur_ms  : 지속 시간 (ms)
 * ═══════════════════════════════════════════════════════════════════════════*/
static void tone(uint16_t freq_hz, uint16_t dur_ms)
{
    if (freq_hz == 0) return;

    /* 샘플당 간격 (μs) = 1,000,000 / (freq × 32) */
    uint32_t sample_us = 1000000UL / ((uint32_t)freq_hz * 32U);

    /* 총 샘플 수 = freq × dur_ms / 1000 × 32 */
    uint32_t total = (uint32_t)freq_hz * dur_ms / 1000U * 32U;

    for (uint32_t i = 0; i < total; i++)
    {
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2,
                         DAC_ALIGN_12B_R, sine32[i & 31]);
        delay_us(sample_us);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  공개 API
 * ═══════════════════════════════════════════════════════════════════════════*/
void Speaker_Init(void)
{
    s_sndQueue = osMessageQueueNew(4, sizeof(uint8_t), NULL);
}

void Speaker_Play(uint8_t sound_id)
{
    if (s_sndQueue)
        osMessageQueuePut(s_sndQueue, &sound_id, 0, 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SpeakerTask  –  FreeRTOS 태스크
 *
 *  큐에서 사운드 ID를 받아 앰프 ON → 톤 출력 → 앰프 OFF
 * ═══════════════════════════════════════════════════════════════════════════*/
void SpeakerTask(void *argument)
{
    (void)argument;
    uint8_t snd;

    for (;;)
    {
        if (osMessageQueueGet(s_sndQueue, &snd, NULL, osWaitForever) == osOK)
        {
            /* 앰프 켜기 + DAC 시작 */
            HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
            HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2,
                             DAC_ALIGN_12B_R, 2048);  /* 중심점 */
            amp_on();
            osDelay(5);   /* TPA2005 startup time (~5 ms) */

            switch (snd)
            {
            case SND_READY:
                /* ▲ 상승 2톤: 1 kHz 100 ms → 2 kHz 100 ms */
                tone(1000, 100);
                tone(2000, 100);
                break;

            case SND_STBY:
                /* ▼ 하강 1톤: 800 Hz 150 ms */
                tone(800, 150);
                break;

            default:
                break;
            }

            /* 앰프 끄기 + DAC 정지 */
            HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2,
                             DAC_ALIGN_12B_R, 2048);  /* pop 방지 */
            osDelay(2);
            amp_off();
            HAL_DAC_Stop(&hdac1, DAC_CHANNEL_2);
        }
    }
}
