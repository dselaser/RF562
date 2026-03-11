/**
 * @file lv_port_disp_templ.c
 *
 */

/*Copy this file as "lv_port_disp.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include <stdio.h>
#include <stdbool.h>
#include "main.h"
#include "cnnx_proj.h"
#include "lvgl/lvgl.h"
#include "lv_port_disp.h"
#include "ST7789V2.h"
#ifdef   __CNNX_FREERTOS_LVGL
#include "cmsis_os2.h"
#endif


/*********************
 *      DEFINES
 *********************/
#define MY_DISP_HOR_RES    240
#define MY_DISP_VER_RES    280

#define  CNNX_LVGL_BUF_LINES   70   /* 10→70: 28회→4회 DMA, 배경전환 7배 가속 */

#ifndef MY_DISP_HOR_RES
    #warning Please define or replace the macro MY_DISP_HOR_RES with the actual screen width, default value 320 is used for now.
    #define MY_DISP_HOR_RES    320
#endif

#ifndef MY_DISP_VER_RES
    #warning Please define or replace the macro MY_DISP_HOR_RES with the actual screen height, default value 240 is used for now.
    #define MY_DISP_VER_RES    240
#endif

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);

static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
//static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//        const lv_area_t * fill_area, lv_color_t color);

/**********************
 *  STATIC VARIABLES
 **********************/
//   for DMA mode
#ifdef    __CNNX_LCD_DMA_ON
static lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
#endif

#if defined(STM32H5)                /*   for STM32H5 or other DCache enabled devices, be careful with Cache Coherency   */
#define   __CNNX_DCACHE_SUPPORT
#pragma message("*** STM32 DCache Support Defined: __CNNX_DCACHE_SUPPORT")
#include "cachel1_armv7.h"
#else
#undef    __CNNX_DCACHE_SUPPORT
#endif

#ifdef CMSIS_OS_H_
static lv_disp_draw_buf_t draw_buf_dsc_2;
#if defined(__CNNX_DCACHE_SUPPORT)  /*   for STM32H5 or other DCache enabled devices, start address must be aligned for DMA   */
static lv_color_t buf_2_1[MY_DISP_HOR_RES * CNNX_LVGL_BUF_LINES] __attribute__((aligned(4)));   /*A buffer for 10 rows*/
static lv_color_t buf_2_2[MY_DISP_HOR_RES * CNNX_LVGL_BUF_LINES] __attribute__((aligned(4)));   /*An other buffer for 10 rows*/
#else
static lv_color_t buf_2_1[MY_DISP_HOR_RES * CNNX_LVGL_BUF_LINES];                        /*A buffer for 10 rows*/
static lv_color_t buf_2_2[MY_DISP_HOR_RES * CNNX_LVGL_BUF_LINES];                        /*An other buffer for 10 rows*/
#endif
#endif


/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_disp_init(void)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    disp_init();

    /*-----------------------------
     * Create a buffer for drawing
     *----------------------------*/

    /**
     * LVGL requires a buffer where it internally draws the widgets.
     * Later this buffer will passed to your display driver's `flush_cb` to copy its content to your display.
     * The buffer has to be greater than 1 display row
     *
     * There are 3 buffering configurations:
     * 1. Create ONE buffer:
     *      LVGL will draw the display's content here and writes it to your display
     *
     * 2. Create TWO buffer:
     *      LVGL will draw the display's content to a buffer and writes it your display.
     *      You should use DMA to write the buffer's content to the display.
     *      It will enable LVGL to draw the next part of the screen to the other buffer while
     *      the data is being sent form the first buffer. It makes rendering and flushing parallel.
     *
     * 3. Double buffering
     *      Set 2 screens sized buffers and set disp_drv.full_refresh = 1.
     *      This way LVGL will always provide the whole rendered screen in `flush_cb`
     *      and you only need to change the frame buffer's address.
     */

    /* Example for 1) */
    // static lv_disp_draw_buf_t draw_buf_dsc_1;
    // static lv_color_t buf_1[MY_DISP_HOR_RES * 10];                          /*A buffer for 10 rows*/
    // lv_disp_draw_buf_init(&draw_buf_dsc_1, buf_1, NULL, MY_DISP_HOR_RES * 10);   /*Initialize the display buffer*/

    /* Example for 2) */
    //  it muse be declared global under FreeRTOS
#ifndef CMSIS_OS_H_
    static lv_disp_draw_buf_t draw_buf_dsc_2;
#if defined(__CNNX_DCACHE_SUPPORT)  /*   for STMH5 or other DCache enabled devices, start address must be aligned for DMA   */
    static lv_color_t buf_2_1[MY_DISP_HOR_RES * CNNX_LVGL_BUF_LINES] __attribute__((aligned(4)));                        /*A buffer for 10 rows*/
    static lv_color_t buf_2_2[MY_DISP_HOR_RES * CNNX_LVGL_BUF_LINES] __attribute__((aligned(4)));                        /*An other buffer for 10 rows*/
#else
    static lv_color_t buf_2_1[MY_DISP_HOR_RES * CNNX_LVGL_BUF_LINES];                      /*A buffer for 10 rows*/
    static lv_color_t buf_2_2[MY_DISP_HOR_RES * CNNX_LVGL_BUF_LINES];                        /*An other buffer for 10 rows*/
#endif
#endif
    lv_disp_draw_buf_init(&draw_buf_dsc_2, buf_2_1, buf_2_2, MY_DISP_HOR_RES * CNNX_LVGL_BUF_LINES);   /*Initialize the display buffer*/

    /* Example for 3) also set disp_drv.full_refresh = 1 below*/
    // static lv_disp_draw_buf_t draw_buf_dsc_3;
    // static lv_color_t buf_3_1[MY_DISP_HOR_RES * MY_DISP_VER_RES];            /*A screen sized buffer*/
    // static lv_color_t buf_3_2[MY_DISP_HOR_RES * MY_DISP_VER_RES];            /*Another screen sized buffer*/
    // lv_disp_draw_buf_init(&draw_buf_dsc_3, buf_3_1, buf_3_2,
    //                      MY_DISP_VER_RES * LV_VER_RES_MAX);   /*Initialize the display buffer*/

    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/

#ifndef   __CNNX_LCD_DMA_ON
    static lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
#endif
    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = MY_DISP_HOR_RES;
    disp_drv.ver_res = MY_DISP_VER_RES;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    /*   BE CAREFUL, draw_buf_dsc_1, draw_buf_dsc_2, or draw_buf_dsc_3 for each buffering mode   */
    disp_drv.draw_buf = &draw_buf_dsc_2;

    /*Required for Example 3)*/
    //disp_drv.full_refresh = 1;

    /* Fill a memory array with a color if you have GPU.
     * Note that, in lv_conf.h you can enable GPUs that has built-in support in LVGL.
     * But if you have a different GPU you can use with this callback.*/
    //disp_drv.gpu_fill_cb = gpu_fill;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);

    printf( "\r\n   ... lv_port_disp_init() : OK\r\n" ) ;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*Initialize your display and the required peripherals.*/
static void disp_init(void)
{
    /*You code here*/
    /*   for ST7789V2   */
    ST7789V2_Init( ST7789V2_VERTICAL );
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/*Flush the content of the internal buffer the specific area on the display
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_disp_flush_ready()' has to be called when finished.*/
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    if(disp_flush_enabled) {
        /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/

        // int32_t x;
        // int32_t y;
        // for(y = area->y1; y <= area->y2; y++) {
        //     for(x = area->x1; x <= area->x2; x++) {
        //         /*Put a pixel to the display. For example:*/
        //         /*put_px(x, y, *color_p)*/
        //         color_p++;
        //     }
        // }

    	//  for lvgl v9.x
        //  lv_draw_sw_rgb565_swap(px_map, ((area->x2 - area->x1 +1)*(area->y2 - area->y1 +1)));
    	// GLCD_Flush(color_p, area->x1, area->y1, area->x2, area->y2);

        // STM32H5 or other DCache enabled devices, Cache Flushing
#ifdef __CNNX_DCACHE_SUPPORT      /*   for STMH5 or other DCache enabled devices, cached memory flush   */
        int   nsize = sizeof(lv_color_t) * (area->y2-area->y1+1) * (area->x2-area->x1+1) ;
        SCB_CleanDCache_by_Addr((uint32_t*)color_p, nsize);
#endif
    	ST7789V2_Flush((UWORD)area->x1, (UWORD)area->y1, (UWORD)area->x2, (UWORD)area->y2, (UWORD *)color_p);
    	// printf(" *** CNNX_Debug : disp_flush() called\r\n") ;

    }

    /*IMPORTANT!!!
     *Inform the graphics library that you are ready with the flushing*/
#ifndef   __CNNX_LCD_DMA_ON
    lv_disp_flush_ready(disp_drv);
#endif
}

#ifdef    __CNNX_LCD_DMA_ON
void GLCD_DMA_FlushReady(void)
{
    /*IMPORTANT!!!
     *Inform the graphics library that you are ready with the flushing*/
	// disp_enable_update() ;
	lv_disp_flush_ready(&disp_drv);
}
#endif


/*OPTIONAL: GPU INTERFACE*/

/*If your MCU has hardware accelerator (GPU) then you can use it to fill a memory with a color*/
//static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//                    const lv_area_t * fill_area, lv_color_t color)
//{
//    /*It's an example code which should be done by your GPU*/
//    int32_t x, y;
//    dest_buf += dest_width * fill_area->y1; /*Go to the first line*/
//
//    for(y = fill_area->y1; y <= fill_area->y2; y++) {
//        for(x = fill_area->x1; x <= fill_area->x2; x++) {
//            dest_buf[x] = color;
//        }
//        dest_buf+=dest_width;    /*Go to the next line*/
//    }
//}


#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif
