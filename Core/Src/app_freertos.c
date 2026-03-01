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

osThreadId_t ads8325AcqTaskHandle;


	// LEDTask
const osThreadAttr_t led_attr = {
	.name       = "1_LEDTask",
	.priority   = osPriorityNormal,
	.stack_size = 512
};

	 // SystemMonitorTask (Heap / Queue 상태 모니터링)
static const osThreadAttr_t mon_attr = {
	.name       = "2_SysMonTask",
	.priority   = osPriorityLow,
	.stack_size = 512*2
};

// ADS7041 UART2 Task
static const osThreadAttr_t ads7041TaskAttr = {
    .name       = "3_ADS7041_UART2",
    .priority   = osPriorityNormal,
    .stack_size = 512*2
};

// ADXL345 엑셀로미터 UART2 Task
static const osThreadAttr_t adxl345TaskAttr = {
    .name       = "4_ADXL345_UART2",
    .priority   = osPriorityNormal,
    .stack_size = 512*2
};

osThreadId_t ads8325TaskHandle;
const osThreadAttr_t ads8325_attr = {
  .name 		= "5_ADS8325_UART2",
  .priority 	= (osPriority_t) osPriorityNormal,
  .stack_size 	= 512*2
};


// HPSwitchTask (VCA 제어)
static const osThreadAttr_t hp_attr = {
  .name       	= "6_HPSwitchTask",
  .priority   	= osPriorityHigh,
  .stack_size 	= 512*3
};

// ADC Task
const osThreadAttr_t ads8325AcqTask_attr = {
  .name 		= "7_ADC_Acq",
  .priority 	= (osPriority_t) osPriorityHigh,
  .stack_size 	= 1024  // bytes
};


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void LEDTask(void *argument);
void SysMonTask(void *argument);
void StartDefaultTask(void *argument);
void HPSwitchTask(void *argument);

void SysMonTask(void *argument);
void ADS8325_RS485_Task(void *argument);
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

	 osThreadNew(ADS7041_UART2_Task, NULL, &ads7041TaskAttr);
	 osThreadNew(ADXL345_UART2_Task, NULL, &adxl345TaskAttr);

	 osThreadNew(ADS8325_RS485_Task, NULL, &ads8325_attr);
	// gUart2TxTaskHandle = ads8325TaskHandle;
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

