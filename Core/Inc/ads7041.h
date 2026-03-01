/*
 * ads7041.h
 *
 *  Created on: Dec 12, 2025
 *      Author: hl3xs
 */

#ifndef __ADS7041_H__
#define __ADS7041_H__

#include "main.h"

// ADS7041는 12bit ADC 이고, SPI로 16bit 프레임을 주고 받습니다.
// 여기서는 12bit 값을 우측 정렬해서 uint16_t 로 돌려줍니다.

uint16_t ADS7041_Read(void);

#endif /* __ADS7041_H__ */

