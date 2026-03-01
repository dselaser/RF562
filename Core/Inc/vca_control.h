
// ===============================
// File: vca_control.h
// ===============================
#ifndef VCA_CONTROL_H
#define VCA_CONTROL_H
#include <stdbool.h>
#include <stdint.h>

// Simple direction enumerations
typedef enum { VCA_PULL = 0, VCA_PUSH = 1 } vca_dir_t; // define according to your mechanics

// API
void VCA_Init(void);
void VCA_Set(vca_dir_t dir, float duty01); // 0..1 duty
void VCA_StopBrake(void);                  // fast stop (0% duty, keep enable)
void VCA_Disable(void);                    // tri-state outputs via DIS


typedef struct {
    float kp;
    float ki;
    float kd;
    float dt;          // 1kHz 적용을 위해 추가한 멤버
    float out_min;
    float out_max;
    float i_acc;       // integral 대신 i_acc로 변경 (에러 해결)
    float prev_e;      // prev_error 대신 prev_e로 변경 (에러 해결)
} PID_t;


#endif
