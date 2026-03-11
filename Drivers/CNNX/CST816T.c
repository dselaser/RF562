/*===========================================================================*\
|=============================================================================|
|                                                                             |
| @file   CST816T.c                                                           |
| @author connexthings@naver.com                                              |
| @brief  CST816T tuouch pad interface for lvgl on STM32                      |
| @version   0.8                                                              |
|                                                                             |
|=============================================================================|
|                                                                             |
| @section  LICENSE                                                           |
|                                                                             |
|   Copyright (c) 2025, connexthings@naver.com                                |
|   All commercial right reserved under the International Copyright Law       |
|                                                                             |
|  This is UNPUBLISHED PROPRIETARY SOURCE CODE of conneXthings.com;           |
|  the contents of this file may not be disclosed to third parties, copied,   |
|  duplicated, deleted or saled in any form, in whole, or in part without     |
|  the prior written permission of the international copyright holder.        |
|                                                                             |
|=============================================================================|
|                                                                             |
|  Usage                                                                      |
|                                                                             |
|=============================================================================|
|                                                                             |
|  Revision History                                                           |
|                                                                             |
|  v0.8  :                                                                    |
|                                                                             |
\*===========================================================================*/

/*****************************************************************************
* | File      	:   Touch_1IN28.c
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*                Used to shield the underlying layers of each master
*                and enhance portability
*----------------
* |	This version:   V1.0
* | Date        :   2022-12-02
* | Info        :   Basic version
*
******************************************************************************/

#include "stdint.h"
#include "main.h"
#include "stm32h5xx_hal.h"
#include "CST816T.h"

#ifndef UBYTE
#define UBYTE   uint8_t
#endif
#ifndef UWORD
#define UWORD   uint16_t
#endif
#ifndef UDOUBLE
#define UDOUBLE uint32_t
#endif

CST816T_XY   _txy ;

extern I2C_HandleTypeDef        hi2c1 ;
#define HI2C                    hi2c1

UBYTE  tp_flag = 0 ;

/******************************************************************************
function:	I2C Function initialization and transfer
parameter:
Info:
******************************************************************************/
// void DEV_I2C_Init(uint8_t Add)
// {
// 	I2C_ADDR =  Add;
// }
//
// CST816T device ID must be 0x15, it is called from Waveshare's Dev_Config.c
// DEV_I2C_Init(0x15 << 1);
#define        TOUCH_ID    (0x15)
static uint8_t _i2c_addr = (TOUCH_ID << 1);

void I2C_Write_Byte(uint8_t Cmd, uint8_t value)
{

	UBYTE Buf[1] = {0};
	Buf[0] = value;
	HAL_I2C_Mem_Write(&HI2C, _i2c_addr, Cmd, I2C_MEMADD_SIZE_8BIT, Buf, 1, 0x20);

}

int I2C_Read_Byte(uint8_t Cmd)
{

	UBYTE Buf[1]={0};
	HAL_I2C_Mem_Read(&HI2C, _i2c_addr+1, Cmd, I2C_MEMADD_SIZE_8BIT, Buf, 1, 0x20);
	return Buf[0];

}

int I2C_Read_Word(uint8_t Cmd)
{

	UBYTE Buf[2]={0, 0};
	HAL_I2C_Mem_Read(&HI2C, _i2c_addr+1, Cmd, I2C_MEMADD_SIZE_8BIT, Buf, 2, 0x20);
	return ((Buf[1] << 8) | (Buf[0] & 0xff));

}

void I2C_Read_nByte(UBYTE Cmd,UBYTE *Buf,UBYTE num)
{
	HAL_I2C_Mem_Read(&HI2C, _i2c_addr+1, Cmd, I2C_MEMADD_SIZE_8BIT, Buf, num, 0x20);

}

/******************************************************************************
function :	read ID 读取ID
parameter:  CST816T : 0xB5
******************************************************************************/
UBYTE _CST816T_WhoAmI()
{

    if (I2C_Read_Byte(0xA7) == 0xB5)   
        return true;
    else
        return false;
}

/******************************************************************************
function :	reset touch 复位触摸
parameter: 
******************************************************************************/
void _CST816T_Reset()
{
    DEV_Digital_Write(DEV_TP_RST_PIN, 0);
    HAL_Delay(100);
    DEV_Digital_Write(DEV_TP_RST_PIN, 1);
    HAL_Delay(100);
}

/******************************************************************************
function :	Read software version number 读取软件版本号
parameter:  
******************************************************************************/
UBYTE _CST816T_ReadRevision()
{
    return I2C_Read_Byte(0xA9);
}

/******************************************************************************
function :	exit sleep mode 退出休眠模式
parameter:  
******************************************************************************/
void _CST816T_StopSleep()
{
    I2C_Write_Byte(DisAutoSleep,0x01);
}

/******************************************************************************
function :	Set touch mode 设置触摸模式
parameter:  
        mode = 0 gestures mode 
        mode = 1 point mode
        mode = 2 mixed mode
******************************************************************************/
void CST816T_SetMode(UBYTE mode)
{
    if (mode == 1)
    {
        I2C_Write_Byte(IrqCtl,0X41);
        I2C_Write_Byte(NorScanPer,0X01);//Normal fast detection cycle unit 10ms   
        I2C_Write_Byte(IrqPluseWidth,0x1c); //Interrupt low pulse output width 1.5MS
    }  else if(mode == 2)
        I2C_Write_Byte(IrqCtl,0X71);
    else {
        I2C_Write_Byte(IrqCtl,0X11);
        I2C_Write_Byte(NorScanPer,0X01);
        I2C_Write_Byte(IrqPluseWidth,0x1c);//中断低脉冲输出宽度 1.5MS
        I2C_Write_Byte(MotionMask,EnDClick);//Enable double-tap mode
     }

}

/******************************************************************************
function :	wake up touchscreen 唤醒触摸屏
parameter:  
******************************************************************************/
void CST816T_WakeUp()
{
    DEV_Digital_Write(DEV_TP_RST_PIN, 0);
    HAL_Delay(10);
    DEV_Digital_Write(DEV_TP_RST_PIN, 1);
    HAL_Delay(50);
    I2C_Write_Byte(0xFE,0x01);
}


/******************************************************************************
function :	screen initialization 屏幕初始化
parameter:  
******************************************************************************/
// UBYTE CST816T_Init(UBYTE mode)
UBYTE CST816T_Init()
{
    UBYTE bRet,Rev;

    _CST816T_Reset();

	while(1) {
		bRet = _CST816T_WhoAmI();			
		if (bRet){
			printf("   ... CST816T detected, ");
			Rev = _CST816T_ReadRevision();
			printf("Revision = %d\r\n",Rev);
			_CST816T_StopSleep();
			break;
		}  else  {
			HAL_Delay(10);
			// It is a static function, call it by proxy function
			// MX_I2C1_Init();
			CNNX_I2C1_Init() ;
			printf("   ... ERROR\007 : CST816T have not detected.\r\n");
            return false ;
		}
	}
    //   set point mode
    _txy.mode = 1 ;
    CST816T_SetMode( 1 );

    _txy.x_point = 0;
    _txy.y_point = 0;
    _txy.gesture = 1;  // default to released state

    // Flush stale touch data after reset
    HAL_Delay(10);
    CST816T_GetPoint();

    return true;
}

/******************************************************************************
function :	Get the corresponding point coordinates 获取对应的点坐标
parameter:  
******************************************************************************/
CST816T_XY CST816T_GetPoint(void)
{
   UBYTE tp_num = 0;
   UBYTE data[4];

   // Read touch point count (register 0x02) first
   I2C_Read_nByte(0x02, &tp_num, 1);
   if(tp_num == 0) {
       // No touch detected - report released state
       _txy.gesture = 1;  // 1 = Lift Up (released)
       return _txy;
   }

   I2C_Read_nByte(0x03, data, 4);

   _txy.gesture = (data[0] & 0xc0) >> 6 ;

   _txy.x_point = ((data[0] & 0x0f)<<8) + data[1];
   _txy.y_point = ((data[2] & 0x0f)<<8) + data[3];

   return _txy ;
}


uint8_t CST816T_Pressed(void)
{
   uint8_t event_reg = 0xFF;

// read register 0x15 or 0x16 depending on your design
// 0x15 usually contains gesture, 0x16 contains event + X high nibble
   // if(HAL_I2C_Mem_Read(&HI2C, _i2c_addr+1, 0x16, I2C_MEMADD_SIZE_8BIT,
   //                  &event_reg, 1, 20) != HAL_OK)
   // {
   //  return 0;   // treat as released on I2C error
   // }
   HAL_I2C_Mem_Read(&HI2C, _i2c_addr+1, 0x01, I2C_MEMADD_SIZE_8BIT, &event_reg, 1, 0x20);
   uint8_t event = event_reg >> 6;  // bits 7..6 = event type
   printf(" tp_evnt : 0x%02X ", event ) ;

// event codes:
// 0 = finger down
// 1 = finger up
// 2 = contact/move
// others: ignore

   if(event == 1 || event == 2)
    return 1;   // pressed or moving

   return 0;       // finger up
}

//
//  It is actually the same as the _I2C_Read_Byte(), but _I2C_Read_Byte() is static function
//
uint8_t CST816T_GetGesture(uint8_t Cmd)
{

	UBYTE Buf[1]={0};
	HAL_I2C_Mem_Read(&HI2C, _i2c_addr+1, Cmd, I2C_MEMADD_SIZE_8BIT, Buf, 1, 0x20);
	return (uint8_t) Buf[0];

}

//
//  touch input interrupt callback was moved here from Waveshare's LCD_1inch69_test.c
//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// if(GPIO_Pin & TP_INT_Pin) {
	/*
	if(GPIO_Pin & TP_IRQ_Pin) {
		tp_flag = TOUCH_IRQ;
		if(_txy.mode == 1) _txy = CST816T_GetPoint();
	}
	*/
}










