/*
 * vca_stub.c
 *
 *  Created on: Dec 14, 2025
 *      Author: hl3xs
 */


#include <stdint.h>

void VCA_PID_SetGains(float kp, float ki, float kd) { (void)kp; (void)ki; (void)kd; }
void VCA_PID_GetGains(float *kp, float *ki, float *kd) { if(kp)*kp=0; if(ki)*ki=0; if(kd)*kd=0; }
void VCA_SetTargetADC(uint16_t adc_target) { (void)adc_target; }
uint16_t VCA_GetTargetADC(void) { return 0; }
