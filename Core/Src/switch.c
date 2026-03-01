/*
 * switch.c
 *
 *  Created on: Dec 26, 2025
 *      Author: hl3xs
 */


#include "switch.h"
#include "main.h"

uint8_t Switch_ReadStable(void)
{
    static uint8_t stable = 0;
    static uint8_t last_raw = 0;
    static uint32_t t_change = 0;

    uint8_t raw =
        (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) ? 1 : 0;

    if (raw != last_raw) {
        last_raw = raw;
        t_change = HAL_GetTick();
    }

    if ((HAL_GetTick() - t_change) >= 20) { // 20 ms debounce
        stable = raw;
    }

    return stable;
}
