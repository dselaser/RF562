/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
#include "uart_cmd.h"
#include "my_tasks.h"
#include "tim.h"

#include "cnnx_proj.h"
#include "lvgl/lvgl.h"
#include "lvgl/demos/lv_demos.h"
#include "ui.h"
#include "lv_port_indev.h"
#include "adxl345.h"
#include "ST7789V2.h"
#include "speaker.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#include "stm32h5xx.h"   // 디바이스 헤더에 맞게
#include "core_cm33.h"   // 보통 포함됨

void ADS8325_AcqTask(void *argument);

void vConfigureTimerForRunTimeStats(void)
{
  // Trace enable (TRCENA)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  // (일부 코어/설정에서 필요할 수 있음) DWT unlock 레지스터가 있는 경우만
  #ifdef DWT_LAR
  DWT->LAR = 0xC5ACCE55;
  #endif

  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   // enable
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

	// LEDTask
const osThreadAttr_t led_attr = {
	.name       = "01_LEDTask",
	.priority   = osPriorityNormal,
	.stack_size = 512
};

	 // SystemMonitorTask (Heap / Queue 상태 모니터링)
static const osThreadAttr_t mon_attr = {
	.name       = "02_SysMonTask",
	.priority   = osPriorityLow,
	.stack_size = 512*2
};

// ADS7041 UART2 Task
static const osThreadAttr_t ads7041TaskAttr = {
    .name       = "03_ADS7041_U2",
    .priority   = osPriorityNormal,
    .stack_size = 512*2
};

// ADXL345 엑셀로미터 UART2 Task (비활성)
static const osThreadAttr_t adxl345TaskAttr = {
    .name       = "04_ADXL345_U2",
    .priority   = osPriorityNormal,
    .stack_size = 512*2
};

// ADXL345 방향 측정 UART1 Task (디버그)
static const osThreadAttr_t adxl345Uart1Attr = {
    .name       = "05_ADXL345_U1",
    .priority   = osPriorityLow,
    .stack_size = 512*3      /* atan2f/sqrtf 사용 → 여유 확보 */
};

// Speaker 사운드 출력 Task
static const osThreadAttr_t speakerAttr = {
    .name       = "09_Speaker",
    .priority   = osPriorityLow,
    .stack_size = 512
};

/*
osThreadId_t ads8325TaskHandle;
const osThreadAttr_t ads8325_attr = {
  .name = "5_ADS8325_UART2",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512*2
};
*/

/* Definitions for ADS8325_Acq */
osThreadId_t ads8325AcqTaskHandle;
const osThreadAttr_t ads8325AcqTask_attr = {
  .name = "07_ADS8325_Acq",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 2
};

/* Definitions for ADS8325_RS485 */
osThreadId_t ads8325TaskHandle;
const osThreadAttr_t ads8325_attr = {
  .name = "06_RS485",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 1
};


// HPSwitchTask (VCA 제어) — float 연산 + TIM6 ISR FPU lazy stacking 고려
static const osThreadAttr_t hp_attr = {
  .name       = "08_HPSwitch",
  .priority   = osPriorityHigh,
  .stack_size = 512*6
};


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for _lvglHandler */
osThreadId_t _lvglHandlerHandle;
const osThreadAttr_t _lvglHandler_attributes = {
  .name = "_lvglHandler",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 2048 * 4
};
/* Definitions for _lvglSmphr */
osSemaphoreId_t _lvglSmphrHandle;
const osSemaphoreAttr_t _lvglSmphr_attributes = {
  .name = "_lvglSmphr"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void LEDTask(void *argument);
void SysMonTask(void *argument);
void StartDefaultTask(void *argument);
void HPSwitchTask(void *argument);

void SysMonTask(void *argument);
void ADS8325_RS485_Task(void *argument);
void ADS8325_AcqTask(void *argument);
// ADS7041 / ADXL345 UART Task 프로토타입
void ADS7041_UART2_Task(void *argument);
void ADXL345_UART2_Task(void *argument);

/* TASK STATS auto-prime (uart_cmd.c) */
extern void UART_Cmd_PrimeTaskStatsBaseline(void);
/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	 UART_Cmd_Init();      // ★ 추가: RX IT 시작 + UARTCmd task 생성 :contentReference[oaicite:2]{index=2}

	 osThreadNew(LEDTask, NULL, &led_attr);
	 osThreadNew(SysMonTask, NULL, &mon_attr);
     osThreadNew(HPSwitchTask, NULL, &hp_attr);
     ads8325AcqTaskHandle = osThreadNew(ADS8325_AcqTask, NULL, &ads8325AcqTask_attr);

	 /* ── 비활성화: HPSwitchTask가 ADS7041 SPI3 직접 읽기하므로
	  *    ADS7041_UART2_Task는 SPI3 + UART2 + CS핀(PD2=TLE9201) 충돌 유발.
	  *    ADXL345_UART2_Task도 UART2 충돌. ──────────────────────────── */
	 // osThreadNew(ADS7041_UART2_Task, NULL, &ads7041TaskAttr);
	 // osThreadNew(ADXL345_UART2_Task, NULL, &adxl345TaskAttr);

	 /* ADXL345 방향 측정 → UART1 콘솔 출력 (I2C2, 충돌 없음) */
	 osThreadNew(ADXL345_UART1_Task, NULL, &adxl345Uart1Attr);

	 ads8325TaskHandle = osThreadNew(ADS8325_RS485_Task, NULL, &ads8325_attr);
	// gUart2TxTaskHandle = ads8325TaskHandle;

	 /* Speaker 사운드 출력 (DAC1_OUT2 + TPA2005D1) */
	 Speaker_Init();
	 osThreadNew(SpeakerTask, NULL, &speakerAttr);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
  /* creation of _lvglSmphr */
  _lvglSmphrHandle = osSemaphoreNew(1, 1, &_lvglSmphr_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of _lvglHandler */
  _lvglHandlerHandle = osThreadNew(lvglHandler, NULL, &_lvglHandler_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  // 리셋 후 자동 기준점 설정: 't'를 1번만 눌러도 stats 출력되게
  //osDelay(300);
 // UART_Cmd_PrimeTaskStatsBaseline();


  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  osDelay(100);

  }
  /* USER CODE END defaultTask */
}

/* USER CODE BEGIN Header_lvglHandler */
/**
* @brief Function implementing the _lvglHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lvglHandler */
void lvglHandler(void *argument)
{
  /* USER CODE BEGIN _lvglHandler */
#ifdef   __CNNX_FREERTOS_ON
  // lvgl demos
  // lv_demo_widgets();
  //  Squareline Studio UI
  ui_init() ;

  /* ── Splash 화면을 LCD에 렌더링한 후 백라이트 ON (노이즈 방지) ── */
  if (osSemaphoreAcquire(_lvglSmphrHandle, osWaitForever) == osOK) {
      lv_timer_handler();                  // Splash 화면 첫 프레임 렌더링
      osSemaphoreRelease(_lvglSmphrHandle);
  }
  DEV_BL_PIN = 800 - 1;                   // 백라이트 80% ON

  lv_port_indev_enable() ;   // enable touch input after UI objects are created
#endif


  /* ── 화면 자동 회전 / 스위치 배경색 상태 추적 ──────────────── */
  static uint8_t last_rot = 0;
  static uint8_t last_sw  = 0xFF;   /* 초기값: 강제 첫 업데이트 */

  /* Infinite loop */
  for(;;)
  {
    // Acquire GUI mutex if used
  	if( osSemaphoreAcquire(_lvglSmphrHandle, 2) == osOK ) {

      /* ── ADXL345 기반 180° 자동 회전 ────────────────────── */
      uint8_t cur_rot = g_screen_rotated;
      if (cur_rot != last_rot) {
          ST7789V2_SetRotation(cur_rot);          /* MADCTL 변경 (SPI2) */
          lv_obj_invalidate(lv_scr_act());         /* 전체 화면 다시 그리기 */
          last_rot = cur_rot;
      }

      /* ── 스위치 누름 → 배경색 흰/검 전환 (READY 모드만) ──── */
      uint8_t cur_sw = (g_gui_ready && g_sw_state) ? 1 : 0;
      if (cur_sw != last_sw) {
          lv_color_t bg = cur_sw ? lv_color_white() : lv_color_black();
          lv_obj_set_style_bg_color(ui_Screen1, bg, LV_PART_MAIN | LV_STATE_DEFAULT);
          last_sw = cur_sw;
      }

      lv_timer_handler(); // Handles LVGL tasks
      osSemaphoreRelease( _lvglSmphrHandle ) ; // Release mutex
  	}

    osDelay(1);
  }
  /* USER CODE END _lvglHandler */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* Stack overflow hook — UART1 콘솔에 태스크 이름 출력 후 정지 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    /* 최소한의 디버그 출력 (인터럽트 비활성 상태이므로 폴링 전송) */
    extern UART_HandleTypeDef huart1;
    const char msg[] = "\r\n!!! STACK OVERFLOW: ";
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, sizeof(msg)-1, 50);
    HAL_UART_Transmit(&huart1, (uint8_t*)pcTaskName, 16, 50);
    const char nl[] = "\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)nl, 2, 50);
    for (;;) { __NOP(); }
}

/* USER CODE END Application */

