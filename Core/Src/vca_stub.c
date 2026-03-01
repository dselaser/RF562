/*
 * vca_stub.c
 *
 *  Created on: Dec 14, 2025
 *      Author: hl3xs
 */

#include <stdint.h>

/* my_tasks.c 에 정의된 변수 */
extern volatile float g_needle_depth_mm;

/* ADC ↔ mm 변환 상수 (my_tasks.c의 VCA_ADC_HOME, VCA_ADC_PER_MM 과 동일) */
#define STUB_ADC_HOME   3300
#define STUB_ADC_PER_MM 10200.0f

void VCA_PID_SetGains(float kp, float ki, float kd) { (void)kp; (void)ki; (void)kd; }
void VCA_PID_GetGains(float *kp, float *ki, float *kd) { if(kp)*kp=0; if(ki)*ki=0; if(kd)*kd=0; }

void VCA_SetTargetADC(uint16_t adc_target)
{
    /* ADC → mm 변환: mm = (adc - HOME) / PER_MM */
    float mm = (float)((int32_t)adc_target - STUB_ADC_HOME) / STUB_ADC_PER_MM;
    if (mm < 0.0f) mm = 0.0f;
    if (mm > 3.5f) mm = 3.5f;
    g_needle_depth_mm = mm;
}

uint16_t VCA_GetTargetADC(void)
{
    /* mm → ADC 변환 */
    float adc = (float)STUB_ADC_HOME + g_needle_depth_mm * STUB_ADC_PER_MM;
    if (adc < 0.0f) adc = 0.0f;
    if (adc > 65535.0f) adc = 65535.0f;
    return (uint16_t)adc;
}
