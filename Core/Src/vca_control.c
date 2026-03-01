
// ===============================
// File: vca_control.c
// ===============================
#include "vca_control.h"
#include "tle9201.h"
#include "stm32h5xx_hal.h"

void VCA_Init(void){
  TLE9201_Init(TLE9201_SPI_MODE0);
  // H-bridge enable after init
  TLE9201_Enable(true);
}

void VCA_Set(vca_dir_t dir, float duty01){
  TLE9201_SetDir(dir==VCA_PUSH);
  TLE9201_SetPWM_duty(duty01);
}

void VCA_StopBrake(void){
  // For PWM/DIR mode the chip may implement active brake when PWM=0 and certain DIR/inputs;
  // here we simply set duty=0.
  TLE9201_SetPWM_duty(0.0f);
}

void VCA_Disable(void){
  TLE9201_Enable(false);
}
