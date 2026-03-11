/*****************************************************************************
* | File        :   LCD_1in69.c
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :   Used to shield the underlying layers of each master and enhance portability
*----------------
* | This version:   V1.0
* | Date        :   2023-03-15
* | Info        :   Basic version
*----------------
* | File        :   ST7789V2.c
* | Author      :   connexthings@naver.com
* | This version:   V1.1
* | Date        :   2025-11-17
* | Info        :   Adaptation for STM32 lvgl support
* | Copyright   :   copyright of the all improved codes belongs to the author under the international copyright
*
******************************************************************************/

#pragma GCC diagnostic ignored "-Wunused-function"
// #include "DEV_Config.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "cnnx_proj.h"
#include "ST7789V2.h"
// #include "stm32h5xx_hal_spi.h"


ST7789V2_ATTRIBUTES st7789v2 ;

#ifdef __CNNX_LCD_DMA_ON
static int   __dma_busy = 0 ;
#endif

#if defined(__CNNX_LCD_DMA_ON) && !defined(__CNNX_LVGL_ON)
//   temporary DMA buffer for NONE LGVL mode
static UWORD  __pbuf[((MY_DISP_HOR_RES > MY_DISP_VER_RES) ? MY_DISP_HOR_RES : MY_DISP_VER_RES) ] ;
#endif


static int _ST7789V2_Init(void)
{
    DEV_Digital_Write(DEV_DC_PIN, 1);
    DEV_Digital_Write(DEV_CS_PIN, 1);
    DEV_Digital_Write(DEV_RST_PIN, 1);
    // Separated for lvgl, it has a separate touch init,
	// DEV_Digital_Write(DEV_IRQ_PIN, 1);
    // DEV_Digital_Write(DEV_TP_RST_PIN, 1);
    HAL_TIM_PWM_Start(&DEV_BL_TMR, DEV_BL_TCH);
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	// DEV_I2C_Init(0x15 << 1);

    //  백라이트 OFF — LVGL 첫 프레임 렌더링 후 켜짐 (노이즈 방지)
    DEV_BL_PIN = 0 ;
	return 0;
}

#ifndef  __CNNX_LVGL_ON
static void _ST7789V2_Exit(void)
{
    DEV_Digital_Write(DEV_DC_PIN, 0);
    DEV_Digital_Write(DEV_CS_PIN, 1);
    //close 
    DEV_Digital_Write(DEV_RST_PIN, 0);
	// DEV_Digital_Write(DEV_IRQ_PIN, 1);
    // DEV_Digital_Write(DEV_TP_RST_PIN, 0);
    HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
}
#endif     //   _CNNX_LVGL_ON

/******************************************************************************
function :  Hardware reset
parameter:
******************************************************************************/
static void _ST7789V2_Reset(void)
{
    ST7789V2_RST_1;
    HAL_Delay(10);
    ST7789V2_RST_0;
    HAL_Delay(20);
    ST7789V2_RST_1;
    HAL_Delay(10);
}

/******************************************************************************
function :  send command
parameter:
     Reg : Command register
******************************************************************************/
static void _ST7789V2_Command(UBYTE Reg)
{
	HAL_StatusTypeDef ret ;
	ST7789V2_CS_1;
    ST7789V2_DC_0;
    HAL_Delay(1) ;
    ST7789V2_CS_0;
    // DEV_SPI_WRITE(Reg);
    ret = HAL_SPI_Transmit(&HLCD_SPI, (uint8_t *)&Reg, 1, 10);
    if( ret != HAL_OK) printf( " SPI_Tx Error: %d, 1 for HAL_Error, 2 for HAL_Busy, 3 for HAL_Timeout @ %s\r\n", (int)ret, __FUNCTION__ ) ;
    // Usually LCD Data will be followed after LCD command,
    // LCD_CS will be raised end of multiple bytes Data Trasfer, not single Data Transfer
    // It may have some benifit in response time
    // ST7789V2_CS_1;
}

/******************************************************************************
function :  send data
parameter:
    Data : Write data
******************************************************************************/
static void _ST7789V2_Data8B(UBYTE Data)
{
	HAL_StatusTypeDef ret ;
    ST7789V2_DC_1;
    // ST7789V2_CS_0;
    // DEV_SPI_WRITE(Data);
    ret = HAL_SPI_Transmit(&HLCD_SPI, (uint8_t *)&Data, 1, 10);
    if( ret != HAL_OK) printf( " SPI_Tx Error: %d, 1 for HAL_Error, 2 for HAL_Busy, 3 for HAL_Timeout @ %s\r\n", (int)ret, __FUNCTION__ ) ;
    // Data Transfer couble be multiple bytes
    // LCD_CS will be raised end of multiple bytes Data Trasfer, not single Data Transfer
    // It may have some benifit in response time
    // ST7789V2_CS_1;
}

/******************************************************************************
function :  send data
parameter:
    Data : Write data
******************************************************************************/
static void _ST7789V2_Data16B(UWORD Data)
{
	HAL_StatusTypeDef ret ;
    ST7789V2_DC_1;
    // ST7789V2_CS_0;
    // DEV_SPI_WRITE((Data >> 8) & 0xFF);
    // DEV_SPI_WRITE(Data & 0xFF);
    ret = HAL_SPI_Transmit(&HLCD_SPI, (uint8_t *)&Data, sizeof(UWORD), 10);
    if( ret != HAL_OK) printf( " SPI_Tx Error: %d, 1 for HAL_Error, 2 for HAL_Busy, 3 for HAL_Timeout @ %s\r\n", (int)ret, __FUNCTION__ ) ;
    // ST7789V2_CS_1;
}

/******************************************************************************
function :  Initialize the lcd register
parameter:
******************************************************************************/
static void _ST7789V2_InitReg(void)
{
    _ST7789V2_Command(0x36);
    _ST7789V2_Data8B(0xC0);

    _ST7789V2_Command(0x3A);
    _ST7789V2_Data8B(0x05);

    _ST7789V2_Command(0xB2);
    _ST7789V2_Data8B(0x0B);
    _ST7789V2_Data8B(0x0B);
    _ST7789V2_Data8B(0x00);
    _ST7789V2_Data8B(0x33);
    _ST7789V2_Data8B(0x35);

    _ST7789V2_Command(0xB7);
    _ST7789V2_Data8B(0x11);

    _ST7789V2_Command(0xBB);
    _ST7789V2_Data8B(0x35);

    _ST7789V2_Command(0xC0);
    _ST7789V2_Data8B(0x2C);

    _ST7789V2_Command(0xC2);
    _ST7789V2_Data8B(0x01);

    _ST7789V2_Command(0xC3);
    _ST7789V2_Data8B(0x0D);

    _ST7789V2_Command(0xC4);
    _ST7789V2_Data8B(0x20);

    _ST7789V2_Command(0xC6);
    _ST7789V2_Data8B(0x13);

    _ST7789V2_Command(0xD0);
    _ST7789V2_Data8B(0xA4);
    _ST7789V2_Data8B(0xA1);

    _ST7789V2_Command(0xD6);
    _ST7789V2_Data8B(0xA1);

    _ST7789V2_Command(0xE0);
    _ST7789V2_Data8B(0xF0);
    _ST7789V2_Data8B(0x06);
    _ST7789V2_Data8B(0x0B);
    _ST7789V2_Data8B(0x0A);
    _ST7789V2_Data8B(0x09);
    _ST7789V2_Data8B(0x26);
    _ST7789V2_Data8B(0x29);
    _ST7789V2_Data8B(0x33);
    _ST7789V2_Data8B(0x41);
    _ST7789V2_Data8B(0x18);
    _ST7789V2_Data8B(0x16);
    _ST7789V2_Data8B(0x15);
    _ST7789V2_Data8B(0x29);
    _ST7789V2_Data8B(0x2D);

    _ST7789V2_Command(0xE1);
    _ST7789V2_Data8B(0xF0);
    _ST7789V2_Data8B(0x04);
    _ST7789V2_Data8B(0x08);
    _ST7789V2_Data8B(0x08);
    _ST7789V2_Data8B(0x07);
    _ST7789V2_Data8B(0x03);
    _ST7789V2_Data8B(0x28);
    _ST7789V2_Data8B(0x32);
    _ST7789V2_Data8B(0x40);
    _ST7789V2_Data8B(0x3B);
    _ST7789V2_Data8B(0x19);
    _ST7789V2_Data8B(0x18);
    _ST7789V2_Data8B(0x2A);
    _ST7789V2_Data8B(0x2E);

    _ST7789V2_Command(0xE4);
    _ST7789V2_Data8B(0x25);
    _ST7789V2_Data8B(0x00);
    _ST7789V2_Data8B(0x00);

    _ST7789V2_Command(0x21);

    _ST7789V2_Command(0x11);
    HAL_Delay(200);
    _ST7789V2_Command(0x29);

    //  End of Data Transfer, Raise the CS line
    ST7789V2_CS_1;
}

/********************************************************************************
function:   Set the resolution and scanning method of the screen
parameter:
        Scan_dir:   Scan direction
********************************************************************************/
static void _ST7789V2_SetAttributes(UBYTE Scan_dir)
{
    // Get the screen scan direction
    st7789v2.screen_mode = Scan_dir;
    UBYTE MemoryAccessReg = 0x00;

    // Get GRAM and LCD width and height
    if (Scan_dir == ST7789V2_HORIZONTAL) {
        st7789v2.height = MY_DISP_HOR_RES ;
        st7789v2.width  = MY_DISP_VER_RES ;
        MemoryAccessReg = 0X70;
    }
    else {
        st7789v2.height = MY_DISP_VER_RES ;
        st7789v2.width  = MY_DISP_HOR_RES ;
        MemoryAccessReg = 0XC0;
    }

    // Set the read / write scan direction of the frame memory
    _ST7789V2_Command(0x36); // MX, MY, RGB mode
    _ST7789V2_Data8B(MemoryAccessReg); // 0x08 set RGB
}

/********************************************************************************
function :  Initialize the lcd
parameter:
********************************************************************************/
void ST7789V2_Init(UBYTE Scan_dir)
{
    // Intialize the LCD
    _ST7789V2_Init();

    // Hardware reset
    _ST7789V2_Reset();

    // Set the resolution and scanning method of the screen
    _ST7789V2_SetAttributes(Scan_dir);

    // Set the initialization register
    _ST7789V2_InitReg();

#ifdef __CNNX_LCD_DMA_ON
    __dma_busy = 0 ;
#endif
}

/********************************************************************************
function:   Sets the start position and size of the display area
parameter:
        x1  :   X direction Start coordinates
        y1  :   Y direction Start coordinates
        x2  :   X direction end coordinates
        y2  :   Y direction end coordinates
********************************************************************************/
void _ST7789V2_SetWindows(UWORD x1, UWORD y1, UWORD x2, UWORD y2)
{
    if (st7789v2.screen_mode == ST7789V2_VERTICAL) {
				// set the X coordinates
        _ST7789V2_Command(0x2A);
        _ST7789V2_Data8B(x1 >> 8);
        _ST7789V2_Data8B(x1);
        _ST7789V2_Data8B((x2-1) >> 8);
        _ST7789V2_Data8B(x2-1);

        // set the Y coordinates
        _ST7789V2_Command(0x2B);
        _ST7789V2_Data8B((y1+20) >> 8);
        _ST7789V2_Data8B(y1+20);
        _ST7789V2_Data8B((y2+20-1) >> 8);
        _ST7789V2_Data8B(y2+20-1);

    }
    else {
        // set the X coordinates
        _ST7789V2_Command(0x2A);
        _ST7789V2_Data8B((x1+20) >> 8);
        _ST7789V2_Data8B(x1+20);
        _ST7789V2_Data8B((x2+20-1) >> 8);
        _ST7789V2_Data8B(x2+20-1);

        // set the Y coordinates
        _ST7789V2_Command(0x2B);
        _ST7789V2_Data8B(y1 >> 8);
        _ST7789V2_Data8B(y1);
        _ST7789V2_Data8B((y2-1) >> 8);
        _ST7789V2_Data8B(y2-1);
    }
    
    _ST7789V2_Command(0X2C);
    // ST7789V2_CS_1;

}

#ifndef  __CNNX_LVGL_ON
/******************************************************************************
function :  Clear screen
parameter:
******************************************************************************/
void ST7789V2_Clear(UWORD clr)
{
    UWORD   uclr ;
    UWORD   i ;
#ifndef __CNNX_LCD_DMA_ON
    UWORD   j ;
#endif
    
    //   bytes swap needed
    uclr = ((clr & 0xFF00) >> 8) | ((clr & 0x00FF) << 8) ;

    _ST7789V2_SetWindows(0, 0, st7789v2.width, st7789v2.height);
    // DEV_Digital_Write(DEV_DC_PIN, 1);
    ST7789V2_DC_1;

#ifdef  __CNNX_LCD_DMA_ON
    memset( __pbuf, uclr, st7789v2.width * sizeof(UWORD));
    for(i = 0; i < st7789v2.height; i++) {
        // DEV_SPI_WRITE((Color>>8)&0xff);
        // DEV_SPI_WRITE(Color);
        // bytes swap here, if it needs
        // uclr = ((Color & 0xFF00) >> 8) + ((Color & 0x00FF) << 8) ;
        // uclr = Color ;
        // _ST7789V2_Data16B( uclr ) ;
        while( __dma_busy ) ;
        __dma_busy = 1 ;
        HAL_SPI_Transmit_DMA(&HLCD_SPI, (uint8_t *)__pbuf, st7789v2.width * sizeof(UWORD) );
    }
#else       // __CNNX_LCD_DMA_ON
    for(i = 0; i < st7789v2.width; i++) {
        for(j = 0; j < st7789v2.height; j++) {
            // DEV_SPI_WRITE((Color>>8)&0xff);
            // DEV_SPI_WRITE(Color);
            // bytes swap here, if it needs
            // uclr = Color ;
            _ST7789V2_Data16B( uclr ) ;
        }
     }
#endif      // __CNNX_LCD_DMA_ON

    ST7789V2_CS_1;
}

/******************************************************************************
function :  Sends the image buffer in RAM to displays
parameter:
******************************************************************************/
void ST7789V2_Display(UWORD *Image)
{
    UWORD i,j, uclr, color ;

    _ST7789V2_SetWindows(0, 0, st7789v2.width, st7789v2.height);
    // DEV_Digital_Write(DEV_DC_PIN, 1);
    ST7789V2_DC_1;
    
    for(i = 0; i < st7789v2.width; i++) {
        for(j = 0; j < st7789v2.height; j++) {
            // DEV_SPI_WRITE((*(Image+i*LCD_1IN69_HEIGHT+j)>>8)&0xff);
            // DEV_SPI_WRITE(*(Image+i*LCD_1IN69_WIDTH+j));
            // bytes swap here, if it needs
            color = *(Image+j*st7789v2.width+i) ;
            uclr = ((color & 0xFF00) >> 8) + ((color & 0x00FF) << 8) ;
            _ST7789V2_Data16B( uclr ) ;
        }
    }

    ST7789V2_CS_1;
}
#endif      //   __CNNX_LVGL_ON/


#ifdef  __CNNX_LVGL_ON
void ST7789V2_Flush(UWORD x1, UWORD y1, UWORD x2, UWORD y2, UWORD *p_img)
{
#ifndef  __CNNX_LCD_DMA_ON    
    // display
    UDOUBLE addr = 0 ;
    UWORD   i, j, uclr ;
#endif
	HAL_StatusTypeDef ret ;
    
    _ST7789V2_SetWindows(x1, y1, x2+1, y2+1);
    ST7789V2_DC_1;
    // green
    // j = 0x07E0 ;
    //   bytes swap needed
    // uclr = ((j & 0xFF00) >> 8) | ((j & 0x00FF) << 8) ;
    // memset( __pbuf, uclr, (x2-x1+1) * sizeof(UWORD));

#ifndef  __CNNX_LCD_DMA_ON
    addr = 0 ;
    for (j = 0; j <= y2 - y1; j++, addr += st7789v2.width) {
        for(i=0;i<x2-x1-1;i++) {
            // DEV_SPI_WRITE((*(Image+Addr+j)>>8)&0xff);
            // DEV_SPI_WRITE(*(Image+Addr+j));
            //uclr = *(p_img+addr+i-x1) ;
            uclr = p_img[addr+i] ;
            // swap byte if needed
            _ST7789V2_Data16B( uclr ) ;
        }
    }
#else
    //   DMA mode will be here
    while( __dma_busy ) ;
    __dma_busy = 1 ;
    ret = HAL_SPI_Transmit_DMA(&HLCD_SPI, (uint8_t *)p_img, (x2-x1+1) * (y2-y1+1)*sizeof(UWORD) );
    if( ret != HAL_OK) printf( " SPI_Tx Error: %d, 1 for HAL_Error, 2 for HAL_Busy, 3 for HAL_Timeout @ %s\r\n", (int)ret, __FUNCTION__ ) ;
  //  HAL_Delay(1) ;
#endif
}

#else       //   __CNNX_LVGL_ON/

void ST7789V2_DrawPoint(UWORD x, UWORD y, UWORD clr)
{
    _ST7789V2_SetWindows( x,y, x,y );
    _ST7789V2_Data16B(clr);
    ST7789V2_CS_1;
}
#endif      //   __CNNX_LVGL_ON/

void ST7789V2_SetBackLight(UWORD val)
{
    DEV_BL_PIN = val ;
}

/********************************************************************************
function:   Set 180° rotation via MADCTL register
parameter:
        rotated:  0 = normal (needle DOWN), 1 = 180° rotated (needle UP)
        MADCTL 0x00 = normal,  0xC0 (MY=1,MX=1) = 180°
        Y-offset (+20) in _ST7789V2_SetWindows is valid for both orientations.
********************************************************************************/
/********************************************************************************
function:   Toggle display color inversion (hardware, instant)
parameter:
        invert_on:  1 = inversion ON (normal), 0 = inversion OFF (colors flipped)
        ST7789V2 cmd 0x21 = INVON,  0x20 = INVOFF
********************************************************************************/
void ST7789V2_SetInversion(uint8_t invert_on)
{
    _ST7789V2_Command(invert_on ? 0x21 : 0x20);
    ST7789V2_CS_1;
}

void ST7789V2_SetRotation(uint8_t rotated)
{
    _ST7789V2_Command(0x36);
    _ST7789V2_Data8B(rotated ? 0x00 : 0xC0);
    ST7789V2_CS_1;
}


#ifdef __CNNX_LCD_DMA_ON

// #else  // defined(STM32H5)
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Deselect when Tx Complete */
  if(hspi == &HLCD_SPI) {
    //  if needed
    // ST7789V2_CS_1;
    __dma_busy = 0 ;
    // notice updating the frame buffer
#ifdef  __CNNX_LVGL_ON
    GLCD_DMA_FlushReady() ;
#endif
  }
}

#endif // __CNNX_LCD_DMA_ON
