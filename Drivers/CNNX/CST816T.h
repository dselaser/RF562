/*****************************************************************************
* | File      	:   Touch_Driver.h
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*                Used to shield the underlying layers of each master 
*                and enhance portability
*----------------
* |	This version:   V1.0
* | Date        :   2022-12-02
* | Info        :   Basic version
*----------------
* | File        :   CST816T.h
* | Author      :   connexthings@naver.com
* | This version:   V1.1
* | Date        :   2025-11-17
* | Info        :   Adaptation for STM32 lvgl support
* | Copyright   :   copyright of the all improved codes belongs to the author under the international copyright
*
******************************************************************************/
#ifndef __Touch_DRIVER_H
#define __Touch_DRIVER_H	
// #include "DEV_Config.h"
#include <stdio.h>
#include <stdlib.h>		//itoa()
#include "main.h"
#include "cnnx_proj.h"

// #undef __CNNX_LVGL_ON
// #define __CNNX_LVGL_ON

// #define address   0x15  //slave address

#define GESTUREID 0x01 //gesture code
#define None 0x00 		//no gesture
#define UP   0x02 		//slide up
#define Down 0x01 		//slide down
#define LEFT 0x03		//Swipe left
#define RIGHT 0x04		//Swipe right
#define CLICK 0x05		//click
#define DOUBLE_CLICK 0x0B//double click
#define LONG_PRESS 0x0C	//Press

#define FingerNum 0X02
#define XposH 0x03 //X coordinate high 4 digits
#define XposL 0x04 //The lower 8 bits of the X coordinate
#define YposH 0x05 //High 4 digits of Y coordinate
#define YposL 0x06 //The lower 8 bits of the Y coordinate

#define BPC0H 0xB0 //High 8 bits of BPC0 value
#define BPC0L 0xB1 //The lower 8 bits of the BPC0 value
#define BPC1H 0xB2 //The upper 8 bits of the BPC1 value
#define BPC1L 0xB3 //The lower 8 bits of the BPC1 value
#define ChipID 0xA7 //Chip model
#define ProjID 0xA8 //Project Number

#define FwVersion 0xA9 //software version number

#define MotionMask 0xEC 
#define EnConLR  0x04 //Enable continuous left and right swipe actions
#define EnConUD  0x02 //Enable continuous up and down sliding action
#define EnDClick 0x01 //Enable double-click action

#define IrqPluseWidth 0xED //Interrupt low pulse output width unit 0.1ms, optional value: 1~200, default value is 10

#define NorScanPer 0xEE //Normal fast detection cycle unit 10ms, optional value: 1~30, default value is 1

#define MotionSlAngle 0xEF //Gesture detection sliding partition angle control Angle=tan(c)*10
							//c is the angle based on the positive direction of the x-axis
#define LpScanRaw1H 0xF0 //Low power consumption scans the upper 8 bits of the reference value of channel 1
#define LpScanRaw1L 0xF1 //Low power scan the lower 8 bits of the reference value of channel 1
#define LpScanRaw2H 0xF2 //Low power consumption scans the upper 8 bits of the reference value of channel 1
#define LpScanRaw2L 0xF3 //Low power scan the lower 8 bits of the reference value of channel 1

#define LpAutoWakeTime 0xF4 //Auto-recalibration cycle at low power consumption Unit: 1 minute, optional value: 1~5. The default value is 5
#define LpScanTH 0xF5 //Low power scan wake-up threshold. The smaller the value, the more sensitive it is. Available values: 1-255. The default value is 48
#define LpScanWin 0xF6 //Low power scan range. The larger the value, the more sensitive and the higher the power consumption. Available values: 0, 1, 2, 3. The default value is 3
#define LpScanFreq 0xF7 //Low power consumption scan frequency The smaller the more sensitive the optional value: 1~255. The default value is 7
#define LpScanIdac 0xF8 //Low power consumption scanning current The smaller the more sensitive Optional value: 1～255
#define AutoSleepTime 0xF9 //When there is no touch within x seconds, it will automatically enter the low power consumption mode. The unit is 1S, and the default value is 2S.

#define IrqCtl 0xFA 
#define EnTest 0x80 //Interrupt pin test, automatically sends out low pulses periodically after enabling
#define EnTouch 0x40 //When a touch is detected, periodically emit a low pulse
#define EnChange 0x20 //Sends a low pulse when a touch state change is detected
#define EnMotion 0x10 //When a gesture is detected, emit a low pulse
#define OnceWLP 0x00 //The long press gesture only sends out a low pulse signal

#define AutoReset 0xFB //When there is a touch but no valid gesture within x seconds, it will automatically reset. The unit 1S is not enabled when it is 0. The default is 5.
#define LongPressTime 0xFC //Press and hold for x seconds to reset automatically. Unit 1S is not enabled when it is 0. The default is 10.
#define IOCtl 0xFD //
#define SOFT_RST 0x04 //Enable soft reset
#define IIC_OD 0x02 //OD
#define En1v8  0x01 //1.8V

#define DisAutoSleep 0xFE //The default is 0, enabling automatic entry into low power consumption mode; non-zero value, prohibiting automatic entry into low power consumption mode 


#define true 1	
#define false 0

#ifndef UBYTE
#define UBYTE   uint8_t
#endif
#ifndef UWORD
#define UWORD   uint16_t
#endif
#ifndef UDOUBLE
#define UDOUBLE uint32_t
#endif

//  all the pin names were inherited form main.h from STM32CubeMX generated symbols
#define DEV_INT_PIN     TP_IRQ_GPIO_Port,TP_IRQ_Pin		//PB6
#define DEV_TP_RST_PIN  TP_RST_GPIO_Port,TP_RST_Pin		//PB5

/**
 * GPIO read and write
**/
#ifndef DEV_Digital_Write
#define DEV_Digital_Write(_pin, _value) HAL_GPIO_WritePin(_pin, _value == 0? GPIO_PIN_RESET:GPIO_PIN_SET)
#endif
#ifndef DEV_Digital_Read
#define DEV_Digital_Read(_pin) HAL_GPIO_ReadPin(_pin)
#endif

typedef struct{
	UBYTE mode;
	UBYTE gesture;
	UWORD color;
	UWORD x_point;
	UWORD y_point;
}  CST816T_XY ;
// }Touch_1IN69_XY;

typedef enum {
   TOUCH_INIT  = 0,
   TOUCH_IRQ,
   TOUCH_FUNCTION,
	TOUCH_DRAW,
	TOUCH_OUT_GESTURE,
	TOUCH_NO_DRAW,
} Touch_STATE;

// extern UBYTE tp_flag = 0,flgh = 0;

// UBYTE Touch_1IN69_init(UBYTE mode);
UBYTE CST816T_Init();
void  CST816T_SetMode(UBYTE mode);
uint8_t CST816T_GetGesture(uint8_t Cmd) ;
// Touch_1IN69_XY Touch_1IN69_Get_Point(void);
CST816T_XY CST816T_GetPoint(void);

#endif
