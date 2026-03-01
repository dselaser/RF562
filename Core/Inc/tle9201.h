/*
 * tle9201.h
 *
 *  Created on: Oct 21, 2025
 *      Author: hl3xs
 */
// ===============================
// File: tle9201.h
// ===============================
#ifndef TLE9201_H
#define TLE9201_H
#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#pragma once
#include <stdint.h>
uint32_t TLE92xx_ReadREV(void);
uint32_t TLE92xx_ReadDIAG(void);


// --- Pin bindings (adjust to your CubeMX-generated symbols) ---
// DIS: PC7, DIR: PC8, CSN: PD2, PWM: PC6 (TIM8_CH1)
#define TLE9201_DIS_GPIO_Port GPIOC
#define TLE9201_DIS_Pin       GPIO_PIN_7

#define TLE9201_DIR_GPIO_Port GPIOC
#define TLE9201_DIR_Pin       GPIO_PIN_8

#define TLE9201_CSN_GPIO_Port GPIOD        //SPI6_CS
#define TLE9201_CSN_Pin       GPIO_PIN_2

// Use SPI6 (PC12 SCK, PA8 MISO, PA7 MOSI)
extern SPI_HandleTypeDef hspi6;  // provided by CubeMX
extern TIM_HandleTypeDef htim8;  // PC6 = TIM8_CH1

// ---------------- SPI command helpers (16-bit words) ----------------
// The exact bitfields depend on the datasheet; keep generic wrappers here.
typedef enum {
  TLE9201_SPI_MODE0 = 0, // CPOL=0, CPHA=0
  TLE9201_SPI_MODE1 = 1  // CPOL=0, CPHA=1 (fallback)
} tle9201_spi_mode_t;

void     TLE9201_Init(tle9201_spi_mode_t preferred_mode);
uint16_t TLE9201_SPI_TxRx(uint16_t tx);
uint16_t TLE9201_ReadRev(void);         // RD_REV after power-up
uint16_t TLE9201_ReadDiag(void);        // Example: read diagnostic/status
void     TLE9201_ClearErrors(void);     // Example: clear latched faults

// ---------------- H-bridge control ----------------
void TLE9201_Enable(bool en);           // DIS pin: en=true -> enable H-bridge
void TLE9201_SetDir(bool forward);      // DIR pin
void TLE9201_SetPWM_duty(float duty01); // 0.0..1.0 on TIM8_CH1 (20 kHz)

#endif // TLE9201_H
