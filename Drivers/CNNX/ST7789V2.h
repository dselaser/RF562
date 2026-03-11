/*****************************************************************************
* | File        :   LCD_1IN69.h
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :   Used to shield the underlying layers of each master and enhance portability
*----------------
* | This version:   V1.0
* | Date        :   2023-03-15
* | Info        :   Basic version
*----------------
* | File        :   ST7789V2.h
* | Author      :   connexthings@naver.com
* | This version:   V1.1
* | Date        :   2025-11-17
* | Info        :   Adaptation for STM32 lvgl support
* | Copyright   :   copyright of the all improved codes belongs to the author under the international copyright
*
******************************************************************************/
#ifndef __ST7789V2_H__
#define __ST7789V2_H__   
    
// #include "DEV_Config.h"
#include <stdint.h>

#include <stdlib.h>     //itoa()
#include <stdio.h>

// from "DEV_Config.h"
#include "main.h"
// #ifdef   __CNNX_FREERTOS_ON
// #include "cmsis_os2.h"
// #endif

// #undef __CNNX_LVGL_ON
// #define __CNNX_LVGL_ON
// #undef  __CNNX_LCD_DMA_ON
// #define __CNNX_LCD_DMA_ON

#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

/**
 * GPIO config
**/
//  all the pin names were inherited form main.h from STM32CubeMX generated symbols
#define DEV_RST_PIN     LCD_RST_GPIO_Port,LCD_RST_Pin		// P??
#define DEV_DC_PIN      LCD_DC_GPIO_Port,LCD_DC_Pin			// P??
#define DEV_CS_PIN		LCD_CS_GPIO_Port,LCD_CS_Pin			// P??

//   for LCD backlight by Timer1 Ch.3 PWM
// extern TIM_HandleTypeDef htim1, htim2, htim3;
// extern TIM_HandleTypeDef htim1, htim2;

// #define DEV_BL_PIN		TIM2->CCR1 							//PA0
// #define DEV_BL_PIN		TIM3->CCR2 							//PA7
// #define DEV_BL_PIN		TIM1->CCR3 							//PA10
// #define DEV_BL_TMR      htim1								//  LCD Back Light PWM timer handle
// #define DEV_BL_TCH      TIM_CHANNEL_3						//  LCD Back Light PWM timer channel

/**
 * GPIO read and write
**/
#define DEV_Digital_Write(_pin, _value) HAL_GPIO_WritePin(_pin, _value == 0? GPIO_PIN_RESET:GPIO_PIN_SET)
#define DEV_Digital_Read(_pin) HAL_GPIO_ReadPin(_pin)


/**
 * PWM_BL
**/
#define DEV_Set_PWM(_Value)     DEV_BL_PIN= _Value

// void DEV_SPI_WRite(UBYTE _dat);
// int DEV_Module_Init(void);
// static int _ST7789V2_Init(void);
// void DEV_Module_Exit(void);
// static void _ST7789V2_Exit(void);

// End of "DEV_Config.h"

// Define handle of SPI port connected to the display
// extern SPI_HandleTypeDef            hspi2 ;
// #define HLCD_SPI                    hspi2

// #define DEV_SPI_WRITE(_dat)    HAL_SPI_Transmit(&HLCD_SPI, (uint8_t *)&_dat, 1, 500)

// #define LCD_1IN69_HEIGHT 280
// #define LCD_1IN69_WIDTH 240
#define MY_DISP_HOR_RES    240
#define MY_DISP_VER_RES    280

#define ST7789V2_HORIZONTAL 0
#define ST7789V2_VERTICAL   1

#define ST7789V2_CS_0  DEV_Digital_Write(DEV_CS_PIN, 0)     
#define ST7789V2_CS_1  DEV_Digital_Write(DEV_CS_PIN, 1)    
                     
#define ST7789V2_RST_0 DEV_Digital_Write(DEV_RST_PIN, 0)
#define ST7789V2_RST_1 DEV_Digital_Write(DEV_RST_PIN, 1)   
                      
#define ST7789V2_DC_0  DEV_Digital_Write(DEV_DC_PIN, 0)
#define ST7789V2_DC_1  DEV_Digital_Write(DEV_DC_PIN, 1)
                      
typedef struct{
    UWORD width ;
    UWORD height ;
    UBYTE screen_mode ;
}  ST7789V2_ATTRIBUTES;
extern ST7789V2_ATTRIBUTES st7789v2 ;

/********************************************************************************
function:   Macro definition variable name
********************************************************************************/
void ST7789V2_Init(UBYTE Scan_dir);
void ST7789V2_Flush(UWORD x1, UWORD y1, UWORD x2, UWORD y2, UWORD *p_img);
void ST7789V2_SetBackLight(UWORD val);
void ST7789V2_SetInversion(uint8_t invert_on);
void ST7789V2_SetRotation(uint8_t rotated);
#ifndef  __CNNX_LVGL_ON
void ST7789V2_Clear(UWORD clr);
void ST7789V2_Display(UWORD *Image);
void ST7789V2_DrawPoint(UWORD x, UWORD y, UWORD clr);
#else
void GLCD_DMA_FlushReady(void) ;
#endif     //   __CNNX_LVGL_ON

#endif   //  __ST7789V2_H__
