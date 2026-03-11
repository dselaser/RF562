/*
 * speaker.h
 *
 *  Created on: Mar 2026
 *      Author: hl3xs
 *
 *  TPA2005D1 + DAC1_OUT2(PA5) 스피커 사운드 출력
 *  SHUTDOWN pin: PC5 (HIGH=ON, LOW=OFF)
 */

#ifndef SPEAKER_H
#define SPEAKER_H

#include <stdint.h>

/* ── 사운드 ID ──────────────────────────────────────────── */
#define SND_READY   1   /* STBY → READY 전환 시 */
#define SND_STBY    2   /* READY → STBY 전환 시 */

/* ── API ────────────────────────────────────────────────── */
void Speaker_Init(void);               /* 큐 생성 (MX_FREERTOS_Init에서 호출) */
void Speaker_Play(uint8_t sound_id);   /* 비동기 사운드 요청 */

/* ── FreeRTOS Task ──────────────────────────────────────── */
void SpeakerTask(void *argument);

#endif /* SPEAKER_H */
