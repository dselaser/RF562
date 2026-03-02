/* ═══════════════════════════════════════════════════════════════════════════════
 *  ///my_tasks.c  –  Loop-1(TIM6, 5 kHz 전류제어) + Loop-2(FreeRTOS, 위치제어)
 *
 *  ┌─────────────────────────────────────────────────────────────────────────┐
 *  │  Loop-2  (FreeRTOS HPSwitchTask, 1 ms)                                 │
 *  │    PA8 스위치 디바운스 → ADS8325 위치 읽기 → 상태머신(APPROACH/MOVE/   │
 *  │    HOLD/RETURN) → g_i_target_mA 를 루프1 에 넘겨줌                     │
 *  │                                                                         │
 *  │  Loop-1  (TIM6 ISR, 200 µs = 5 kHz)                                   │
 *  │    ADS7041 SPI3 로 전류 읽기 → PI 제어 → TIM8 PWM Duty 직접 수정      │
 *  │    g_i_target_mA (루프2 지령) + 포화 방지 + HOLD 절전                  │
 *  └─────────────────────────────────────────────────────────────────────────┘
 *
 *  ─ 전류 감지 회로 ─────────────────────────────────────────────────────────
 *    Shunt        : R9 = 10 mΩ  (2512 1 %)
 *    INA240A1     : 게인 20 V/V
 *    REF1933      : 바이어스 1.647 V  (ADS7041 AINM 기준)
 *    ADS7041      : 12 bit, Vref = 3.3 V  → 1 LSB = 3.3/4096 V
 *
 *    ADC 전압 = 1.647 + I_vca × 0.010 × 20
 *    → I_vca (A) = (ADC_V − 1.647) / 0.20
 *    → I_vca (mA) = (ADC_code − BIAS_CODE) × (3300/4096) / 0.20
 *                 = (ADC_code − BIAS_CODE) × 4.028 mA/LSB
 *
 *  ─ TIM6 설정 지침 (CubeMX 에서 직접 추가) ─────────────────────────────────
 *    Prescaler     = 0  (카운터 클록 = APB1Tim = 50 MHz)
 *    Counter Period = 9999  → 50 MHz / (0+1) / (9999+1) = 5 000 Hz
 *    Enable TIM6 global interrupt, Priority = 5 (FreeRTOS 보다 높게)
 *    ※ CubeMX 에서 "TIM6 global interrupt" 를 활성화하면
 *      stm32h5xx_it.c 에 TIM6_DAC_IRQHandler 가 자동 생성됩니다.
 *      그 핸들러 안에 HAL_TIM_IRQHandler(&htim6) 호출이 들어갑니다.
 *
 *  ─ SPI3 설정 (CubeMX, ADS7041) ──────────────────────────────────────────
 *    이미 IOC 에 SPI3 이 설정되어 있음 (4.17 MHz, CPOL=0/CPHA=1, 16 bit)
 *    CS 핀 : PA15 (SPI3_CS, GPIO Output)
 * ═══════════════════════════════════════════════════════════════════════════*/

#include "my_tasks.h"
#include "usart.h"
#include "main.h"
#include "spi.h"
#include "tim.h"          // htim6, htim8 extern
#include "ads8325.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tle9201.h"
#include "vca_control.h"
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "switch.h"

size_t UART_Cmd_GetRxQueueFree(void);
size_t UART_Cmd_GetRxQueueLength(void);

/* Loop-1 ISR 핸들러 – main.c 의 HAL_TIM_PeriodElapsedCallback 에서 호출 */
void Loop1_TIM6_ISR_Handler(void);

/* ═══════════════════════════════════════════════════════════════════════════
 *  공유 전역 변수
 * ═══════════════════════════════════════════════════════════════════════════*/

/* Loop-2 → Loop-1 지령 (단위: mA, 양수=PUSH, 음수=PULL) */
volatile int32_t g_i_target_mA = 0;

/* Loop-1 → Loop-2 모니터링 */
volatile int32_t  g_i_meas_mA   = 0;   // 실측 전류 (mA)
volatile uint16_t g_adc7041_raw = 0;   // ADS7041 원시값 (디버그)
volatile uint8_t  g_i_meas_updated = 0; // Loop2가 새 전류값 쓸 때 1, ISR이 읽은 후 0

/* Loop-2 위치 */
volatile uint16_t g_vca_pos_adc = 0;   /* 제어+표시용 (모터 중 동결) */
volatile uint16_t g_vca_pos_raw = 0;   /* median only  (디버그) */
volatile int32_t  g_vca_err     = 0;
volatile uint8_t  g_motor_active = 0;  /* 1=모터 PWM 중 → ADC 동결 */

/* 니들 삽입 깊이 (0.5 ~ 3.5 mm, 런타임 변경 가능) */
volatile float    g_needle_depth_mm = 1.5f;  /* ← VCA_DEPTH_MM_MAX 와 일치시킬 것 */
/* PROBE 결과: 0=무부하, 1=유부하(피부 감지) */
volatile uint8_t  g_load_detected = 0U;

/* Debug */
volatile float   g_u_dbg    = 0;
volatile float   g_duty_dbg = 0;
volatile int     g_dir_dbg  = 0;
volatile int     g_en_dbg   = 0;
/* ISR 호출 카운터 (TIM6 동작 확인용) */
volatile uint32_t g_isr_cnt  = 0;
volatile uint8_t  g_sw_state = 0;
volatile int32_t  g_blk_remain = 0;  /* release block 잔여시간 디버그 */
volatile uint16_t g_sm_pos    = 0;   /* 상태머신이 실제로 보는 pos */
volatile int      g_state_dbg = 0;

/* UART */
TaskHandle_t gUart2TxTaskHandle = NULL;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
static HAL_StatusTypeDef Uart1_Tx_IT(uint8_t *pData, uint16_t size);
static HAL_StatusTypeDef Uart2_Tx_IT(uint8_t *pData, uint16_t size);

/* ADS8325 (위치 ADC) */
#ifndef ADS8325_SAMPLES_PER_LINE
#define ADS8325_SAMPLES_PER_LINE  5
#endif
static uint16_t g_ads8325_samples[ADS8325_SAMPLES_PER_LINE];
static char     g_ads8325_buf[256];

/* ═══════════════════════════════════════════════════════════════════════════
 *  Loop-1 파라미터 – 전류 제어 (TIM6 ISR)
 * ═══════════════════════════════════════════════════════════════════════════*/

/* ─── ADS7041 전류 변환 상수 ─────────────────────────────────────────────
 *  실측 바이어스: VCA 전류=0 일 때 R(raw) = 639~640
 *  → BIAS_CODE = 640
 *  mA/LSB = (3300mV/4096) / (20gain × 10mΩ) = 4.028 mA/LSB (동일)
 * ─────────────────────────────────────────────────────────────────────────*/
#define ADC7041_BIAS_CODE    (640)         /* 실측값 (구: 2043 → 수정) */
#define ADC7041_MA_PER_LSB_F (4.028f)      /* mA / LSB                  */

/* ─── 전류 한계 ──────────────────────────────────────────────────────────*/
#define I_BOOST_MA           (7000)   /* PUSH 부스트 전류 한계 (7 A)        */
#define I_HOLD_MA            (7000)   /* PUSH 구간 상한 (HOLD에서 Loop1 OFF) */
#define I_MAX_MA             (8000)   /* 절대 최대 (8 A, 하드 리미트)       */

/* ─── TIM8 ARR은 TLE9201_Init()이 40kHz로 변경 (50MHz/40kHz-1 = 1249) ────
 *  Loop-1 ISR은 TIM8->ARR을 실시간으로 읽어 CCR을 계산함 (고정값 사용 금지)
 * ─────────────────────────────────────────────────────────────────────────*/

/* ─── 전류 PI 게인 ───────────────────────────────────────────────────────
 *  5 kHz 루프 = dt 0.2 ms
 *  Kp: duty 1%당 약 50 mA 변화 기준 → Kp ≈ 0.02 / 50 = 0.0004
 *  Ki: 전기 시정수 Te = 0.25 ms  →  Ki ≈ Kp / Te = 1.6  (dt 반영 후 작게)
 *  처음에는 아래 값으로 시작, 필요 시 조정
 * ─────────────────────────────────────────────────────────────────────────*/
#define ILOOP_KP             (0.00040f) /* PI 게인 — 1ms 갱신 기준 */
#define ILOOP_KI             (0.00010f) /* 적분 게인 × dt 이미 포함 */
#define ILOOP_IMAX           (0.60f)      /* 적분 클램프 (duty 단위)   */

/* ─── Loop-1 내부 상태 ───────────────────────────────────────────────────*/
static volatile float  s_i_err_acc  = 0.0f;  /* PI 적분 누적        */
static volatile float  s_duty_l1    = 0.0f;  /* Loop-1 현재 duty    */
static volatile int8_t s_l1_dir     = 1;     /* +1=PUSH, -1=PULL    */
static volatile bool   s_l1_enable  = false; /* Loop-1 활성화 여부  */

/* ─── 부스트 시간 제한 ───────────────────────────────────────────────────
 *  부스트(고전류)는 처음 BOOST_TICKS (= 부스트ms × 5) 만큼만 허용
 *  이후 I_HOLD_MA 로 자동 하향
 * ─────────────────────────────────────────────────────────────────────────*/
#define BOOST_TICKS          (250U)   /* 250 ticks × 0.2ms = 50 ms 부스트  */
static volatile uint32_t s_boost_cnt  = 0;
static volatile bool     s_boosting   = false;

/* ADS7041 SPI3 CS 핀 (IOC: PA15 = SPI3_CS) */
#define ADS7041_CS_PORT   GPIOA
#define ADS7041_CS_PIN    GPIO_PIN_15

/* ─── ADS7041 읽기 함수 (SPI3, HAL 폴링 – 태스크에서만 호출) ──────────
 *  ★ ISR 에서 절대 호출 금지
 *  ADS7041: CS LOW → 16클럭 → bit[15:4] = 12bit ADC 결과
 * ─────────────────────────────────────────────────────────────────────*/
extern SPI_HandleTypeDef hspi3;   /* spi.c 에서 선언 */

static uint16_t ADS7041_ReadRaw(void)
{
    uint16_t tx = 0x0000U;
    uint16_t rx = 0x0000U;

    HAL_GPIO_WritePin(ADS7041_CS_PORT, ADS7041_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&tx, (uint8_t*)&rx, 1, 5);
    HAL_GPIO_WritePin(ADS7041_CS_PORT, ADS7041_CS_PIN, GPIO_PIN_SET);

    return (rx >> 4U);   /* bit[15:4] → 12bit 결과 */
}

/* ─── TIM8 Duty 직접 설정 (ISR 에서 안전) ───────────────────────────────
 *  ★ ARR은 TLE9201_Init()이 40kHz로 변경하므로 고정값 사용 금지
 *    → 매번 TIM8->ARR을 읽어서 계산
 * ─────────────────────────────────────────────────────────────────────────*/
static inline void TIM8_SetDutyDirect(float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    uint32_t arr = TIM8->ARR;   /* 실시간 ARR 읽기 (TLE9201_Init이 변경함) */
    TIM8->CCR1 = (uint32_t)(duty * (float)(arr + 1U));
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Loop-1 : TIM6 5 kHz 전류 제어 핸들러
 *
 *  이 함수를 main.c 의 HAL_TIM_PeriodElapsedCallback 안에서
 *  TIM6 인스턴스 확인 후 호출하세요:
 *
 *    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 *    {
 *        if (htim->Instance == TIM6) { Loop1_TIM6_ISR_Handler(); }
 *        // htim1 등 기존 코드...
 *    }
 *
 *  ┌──────────────────────────────────────────────────────────────────────┐
 *  │  1) ADS7041 로 실제 전류 읽기                                        │
 *  │  2) g_i_target_mA (Loop-2 지령) → 한계 클램프 적용                  │
 *  │     ① 부스트 구간(처음 50 ms): I_BOOST_MA 까지 허용                 │
 *  │     ② 이후 HOLD 구간: I_HOLD_MA 로 제한                             │
 *  │  3) PI 제어 → duty 계산 → TIM8 CCR1 직접 수정                       │
 *  └──────────────────────────────────────────────────────────────────────┘
 * ═══════════════════════════════════════════════════════════════════════════*/
void Loop1_TIM6_ISR_Handler(void)
{
    g_isr_cnt++;   /* TIM6 ISR 호출 확인용 – UART에서 증가하면 정상 */

    /* ★ ISR 안에서 SPI/HAL 호출 금지 – FreeRTOS 전체를 블록시킴
     *   전류 측정(ADS7041)은 Loop-2 태스크(HPSwitchTask)에서 수행하고
     *   g_i_meas_mA 에 저장 → ISR 은 그 값만 읽음                       */

    /* ── 1. 루프 비활성화 시 → duty는 Loop-2가 관리, ISR은 손대지 않음 ── */
    if (!s_l1_enable) {
        s_i_err_acc = 0.0f;
        s_duty_l1   = 0.0f;
        return;
    }

    /* ── 3. 지령 전류 한계 적용 ─────────────────────────────────────── */
    int32_t i_tgt = g_i_target_mA;

    /* 방향 결정 */
    if (i_tgt >= 0) {
        s_l1_dir = 1;   /* PUSH */
    } else {
        s_l1_dir = -1;  /* PULL */
        i_tgt    = -i_tgt;
    }

    /* 부스트 카운터 관리 */
    if (s_boosting) {
        if (s_boost_cnt < BOOST_TICKS) {
            s_boost_cnt++;
        } else {
            s_boosting = false;  /* 부스트 종료 → HOLD 한계로 전환 */
        }
    }

    /* 전류 한계 선택 */
    int32_t i_limit = s_boosting ? I_BOOST_MA : I_HOLD_MA;
    if (i_tgt > i_limit) i_tgt = i_limit;
    if (i_tgt > I_MAX_MA) i_tgt = I_MAX_MA;  /* 절대 최대 */

    /* ── 4. PI 전류 제어 ─────────────────────────────────────────────── */
    /* 측정값도 방향 반영 (PUSH 방향 양수 기준) */
    /* ── 4. PI 전류 제어 — 새 측정값 있을 때만 계산 (1ms 갱신 대응) ── */
    if (g_i_meas_updated) {
        g_i_meas_updated = 0;  /* 플래그 클리어 */

        int32_t i_meas_dir = (s_l1_dir == 1) ? g_i_meas_mA : -g_i_meas_mA;
        float   err_f      = (float)(i_tgt - i_meas_dir);

        /* 비례항 */
        float up = ILOOP_KP * err_f;

        /* 적분항 (Anti-windup 포함) */
        s_i_err_acc += ILOOP_KI * err_f;
        if      (s_i_err_acc >  ILOOP_IMAX) s_i_err_acc =  ILOOP_IMAX;
        else if (s_i_err_acc < -ILOOP_IMAX) s_i_err_acc = -ILOOP_IMAX;

        float duty_cmd = up + s_i_err_acc;

        /* duty 클램프 */
        if (duty_cmd < 0.0f) duty_cmd = 0.0f;
        if (duty_cmd > 1.0f) duty_cmd = 1.0f;

        s_duty_l1  = duty_cmd;
        g_duty_dbg = duty_cmd;

        /* ── 5. PWM 적용 ──────────────────────────────────────────────── */
        TIM8_SetDutyDirect(duty_cmd);
    }
    /* 새 측정값 없으면 이전 duty 유지 (TIM8 건드리지 않음) */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Loop-1 제어 API  (Loop-2 에서 호출)
 * ═══════════════════════════════════════════════════════════════════════════*/

/**
 * @brief  전류 루프 시작. 부스트 모드(고전류 관통력) 포함
 * @param  i_target_mA  목표 전류 (양수=PUSH, 음수=PULL)
 * @param  boost        true = 처음 BOOST_TICKS 동안 I_BOOST_MA 허용
 */
static void Loop1_Start(int32_t i_target_mA, bool boost)
{
    s_i_err_acc  = 0.0f;
    s_duty_l1    = 0.0f;
    s_boost_cnt  = 0U;
    s_boosting   = boost;
    g_i_target_mA = i_target_mA;
    s_l1_enable  = true;
}

/**
 * @brief  전류 루프 정지 (PWM=0)
 */
static void Loop1_Stop(void)
{
    s_l1_enable   = false;
    g_i_target_mA = 0;
    s_i_err_acc   = 0.0f;
}

/**
 * @brief  목표 전류만 갱신 (부스트 상태는 유지)
 */
static void Loop1_SetTarget(int32_t i_target_mA)
{
    g_i_target_mA = i_target_mA;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Loop-2 위치 제어 파라미터 (기존 코드 유지)
 * ═══════════════════════════════════════════════════════════════════════════*/

#ifndef HP_SW_GPIO_Port
#define HP_SW_GPIO_Port GPIOA
#endif
#ifndef HP_SW_Pin
#define HP_SW_Pin GPIO_PIN_8
#endif
#ifndef DIR_PUSH
#define DIR_PUSH    (1)   /* TLE9201 DIR HIGH = 실제 PUSH (pos 증가) ★최종확정★ */
#endif
#ifndef DIR_PULL
#define DIR_PULL    (0)   /* TLE9201 DIR LOW  = 실제 PULL (pos 감소) ★최종확정★ */
#endif

#define HP_SAMPLE_MS          (1U)
#define HP_DEBOUNCE_MS        (10U)
#define HP_PULL_KICK_MS       (60U)
#define HP_PULL_HOLD1_MS      (120U)
#define HP_PULL_HOLD1_DUTY    (0.12f)
#define HP_PULL_HOLD2_DUTY    (0.04f)
#define HP_PULL_SOFTSTART_MS  (8U)
#define HP_PULL_KICK_MIN_DUTY (0.60f)
#define HP_PULL_TOTAL_MAX_MS  (300U)
#define HP_PUSH_KICK_MS       (100U)
#define HP_PUSH_HOLD_DUTY     (0.20f)
#define DUTY_HOLD_MIN         0.040f

#define HP_STARTUP_PULL_ENABLE    (0)
#define HP_STARTUP_PULL_KICK_MS   (HP_PULL_KICK_MS)
#define HP_STARTUP_PULL_HOLD_MS   (120U)
#define HP_STARTUP_PULL_HOLD_DUTY (HP_PULL_HOLD1_DUTY)

/* ─── 전류 지령 매핑 ─────────────────────────────────────────────────────
 *  Loop-2 는 duty 대신 전류(mA)를 Loop-1 에 지령합니다.
 *  아래 매크로로 각 상태에서 목표 전류를 설정하세요.
 *
 *  VCA 스펙: Fp=44N, Kf=7.6 N/A, Fc=13.8N, I_cont≈1.8A
 *  피부 관통 시 순간 8A (I_BOOST_MA), 위치 유지 시 1~2A
 * ─────────────────────────────────────────────────────────────────────────*/
#define I_CMD_APPROACH_MA    (1000)   /* 미사용 */
#define I_CMD_MOVE_MA        (1000)   /* 미사용 (런타임 결정) */
#define I_CMD_BOOST_MA       (3000)   /* 미사용 */
#define I_CMD_HOLD_PUSH_MA    (400)   /* 미사용 */
#define I_CMD_HOLD_PULL_MA   (-800)   /* 미사용 */
#define I_CMD_RETURN_MA      (-2000)  /* 미사용 */

/* VCA_WRAP 상수 – VCA_StartupPushWrapToZeroOnce() 에서 사용 */
#ifndef VCA_WRAP_HIGH_TH
#define VCA_WRAP_HIGH_TH  (64000U)
#endif
#ifndef VCA_WRAP_LOW_TH
#define VCA_WRAP_LOW_TH   (2000U)
#endif
#ifndef VCA_WRAP_TARGET
#define VCA_WRAP_TARGET   (150U)
#endif

/* ── 위치 ADC 설정 ────────────────────────────────────────────────────────
 *  VCA_ADC_PER_MM : ★ 기구 실측 필수 ★
 *    HOME(pos≈3500)에서 캘리퍼로 1mm 이동 후 ADC 차이 측정
 * ─────────────────────────────────────────────────────────────────────────*/
#define VCA_ADC_HOME          (3300)
#define VCA_ADC_PER_MM        (10200)  /* 실측: (39000-3300)/3.5mm */
#define VCA_DEPTH_MM_MIN      (1.5f)   /* 선형화 유효 최소 깊이 */
#define VCA_DEPTH_MM_MAX      (4.0f)   /* 선형화 유효 최대 깊이 */
#define VCA_ADC_MAX_SAFE      (39000)  /* 40000보다 1000 여유 */
/* ── 깊이 → ADC 변환 ────────────────────────────────────────────────────
 *  실측: ADC_HOME=3300, ADC 39000 = 실제 3.5mm
 *  PER_MM = (39000 - 3300) / 3.5 = 10200
 */
#define DEPTH_MM_TO_ADC(mm)   ((int32_t)(VCA_ADC_HOME + (mm) * VCA_ADC_PER_MM))
#define VCA_ADC_TARGET        DEPTH_MM_TO_ADC(g_needle_depth_mm)

/* ══ 1단계: PROBE — 부하 판별 (~1mm PUSH 후 전류 측정) ══════════════════
 *  PROBE_DUTY      : PROBE duty (작게 — 부하 감지만)
 *  PROBE_ADC_END   : PROBE 끝 위치 (HOME + 1mm)
 *  PROBE_CURRENT_TH: 이 전류(mA) 이상이면 유부하(피부 있음) ★실측 조정★
 *  PROBE_TIMEOUT_MS: PROBE 최대 허용 시간
 * ═════════════════════════════════════════════════════════════════════════*/
#define PROBE_DUTY            (0.280f) /* PROBE duty — 유부하 저항 극복용 */
#define PROBE_ADC_END         ((int32_t)(VCA_ADC_HOME + 1 * VCA_ADC_PER_MM / 2))  /* 0.5mm */
#define PROBE_CURRENT_TH      (270)    /* mA 실측 확정: 무부하 max=240, 유부하=330 */
#define PROBE_TIMEOUT_MS      (300U)   /* 유부하 저항으로 느릴 수 있음 */

/* ══ 2단계: PUSH ═════════════════════════════════════════════════════════
 *  PUSH_I_NO_LOAD  : 무부하 전류 — 낮게 유지해야 오버슈트 없음
 *  PUSH_I_LOADED   : 유부하(피부) 관통 전류
 *  VCA_BRAKE_ZONE  : 목표까지 이 거리부터 PUSH_I_BRAKE로 낮춤
 *                    ★ 크게 잡을수록 일찍 감속 → 오버슈트 감소 ★
 *  PUSH_I_BRAKE    : 제동 구간 전류 (작을수록 부드러운 정지)
 *
 *  무부하 진동    → PUSH_I_NO_LOAD 낮춤, VCA_BRAKE_ZONE 키움
 *  부하 오버슈트  → PUSH_I_LOADED 낮춤, VCA_BRAKE_ZONE 키움
 *  부하 관통력 부족 → PUSH_I_LOADED 올림
 * ═════════════════════════════════════════════════════════════════════════*/
#define PUSH_I_NO_LOAD        (800)    /* mA: 무부하 — (현재 미사용, Loop2 duty 직접제어로 변경) */
#define NO_LOAD_MOVE_DUTY     (0.215f) /* 무부하 MOVE duty */
#define NO_LOAD_BRAKE_DUTY    (0.190f) /* 무부하 1차 제동 */
#define NO_LOAD_BRAKE2_DUTY   (0.165f) /* 무부하 2차 제동 */
#define NO_LOAD_BRAKE_ZONE2   (3000)   /* 2차 제동 시작 거리 */
#define PUSH_I_LOADED         (6000)   /* mA: 유부하 피부관통 전류 (참고용) */
#define PUSH_DUTY_LOADED      (0.450f) /* 유부하 PUSH duty */
#define VCA_BRAKE_ZONE        (8000)  /* ADC: 목표 8000 전부터 제동 */
#define PUSH_I_BRAKE          (300)    /* mA: 제동 전류 — 무부하 부드러운 정지 */

/* ══ 3단계: HOLD — Loop2 단독 duty 직접제어 ══════════════════════════════
 *  Loop1 완전 OFF. 발열 없는 최소 duty로 스프링 평형 유지.
 *
 *  HOLD_DUTY_FF : 스프링(Fc=13.8N) 버티는 duty
 *    pos 서서히 내려감 → 올림 (0.01 단위)
 *    pos 서서히 올라감 → 낮춤
 *  HOLD_KP      : 데드밴드 밖 P 보정 (작게)
 *  VCA_DB_HOLD  : 이 범위 안에서 FF duty만 → 진동 없음 (핵심)
 * ═════════════════════════════════════════════════════════════════════════*/
#define HOLD_DUTY_FF          (0.360f) /* 유부하 실리콘 반발력 극복 */
#define HOLD_DUTY_MAX         (0.60f)  /* HOLD/MOVE 최대 duty */
#define HOLD_KP               (0.000025f) /* P보정 */
#define VCA_DB_HOLD           (2000)      /* 데드밴드 좁혀서 빠른 보정 */
#define VCA_DB_MOVE           (400)

/* ══ 선형화 LUT: 원하는 깊이(mm) → 필요한 duty ═══════════════════════════
 *  2차 측정 데이터 (선형화 1차 적용 후):
 *    G100 (duty 0.2026) → 1.65mm
 *    G150 (duty 0.2079) → 2.00mm
 *    G200 (duty 0.2132) → 2.33mm
 *    G250 (duty 0.2203) → 2.52mm
 *    G300 (duty 0.2293) → 2.90mm
 *    G350 (duty 0.2382) → 3.28mm
 *    G400 (duty 0.3516) → 4.00mm
 *
 *  역변환: desired_depth_mm → required_duty
 *  구간별 선형 보간으로 비선형 VCA 특성 보상
 * ═══════════════════════════════════════════════════════════════════════*/
#define LIN_LUT_N  8
static const float lut_depth[LIN_LUT_N] = {
    0.00f, 1.65f, 2.00f, 2.33f, 2.52f, 2.90f, 3.28f, 4.00f
};
static const float lut_duty[LIN_LUT_N] = {
    0.192f, 0.2026f, 0.2079f, 0.2132f, 0.2203f, 0.2293f, 0.2382f, 0.3516f
};

static float depth_to_duty(float depth_mm)
{
    /* 범위 밖 클램프 */
    if (depth_mm <= lut_depth[0])
        return lut_duty[0];
    if (depth_mm >= lut_depth[LIN_LUT_N - 1])
        return lut_duty[LIN_LUT_N - 1];

    /* 구간별 선형 보간 */
    for (int i = 0; i < LIN_LUT_N - 1; i++) {
        if (depth_mm <= lut_depth[i + 1]) {
            float t = (depth_mm - lut_depth[i])
                    / (lut_depth[i + 1] - lut_depth[i]);
            return lut_duty[i] + t * (lut_duty[i + 1] - lut_duty[i]);
        }
    }
    return lut_duty[LIN_LUT_N - 1];
}
#define MOVE_TIMEOUT_MS       (500)   /* MOVE 시간 제한: ADC 동결 중 위치 전환 불가 → 타이머로 HOLD 전환 */

/* ══ 4단계: RETURN ═══════════════════════════════════════════════════════
 *  전략: PULL은 먼 거리에서만 약하게. HOME 근처는 스프링 복원력만
 *        사용하되, PUSH 역제동(능동 브레이크)으로 부드럽게 착륙.
 *
 *  ┌── dist > RETURN_COAST_ZONE ──┐  약한 PULL (스프링 보조)
 *  ├── BRAKE_ZONE < dist ≤ COAST ─┤  FREE: duty=0, 스프링만
 *  ├── STOP_POS < dist ≤ BRAKE    ┤  PUSH 역제동 (거리+속도 비례)
 *  └── dist ≤ STOP_POS ──────────┘  PUSH 쿠션 → 0 페이드아웃
 * ═══════════════════════════════════════════════════════════════════════*/
#define RETURN_FAST_DUTY      (0.080f) /* 먼 거리 PULL duty (약하게) */
#define RETURN_COAST_ZONE     (20000)  /* HOME+20000 이상에서만 PULL */
#define RETURN_BRAKE_ZONE     (12000)  /* HOME+12000 이내: PUSH 역제동 시작 */
#define RETURN_BRAKE_VEL_K    (0.0012f) /* 속도 비례 제동 게인 */
#define RETURN_BRAKE_POS_MAX  (0.20f)  /* 위치 비례 제동 최대 duty */
#define RETURN_BRAKE_TOTAL_MAX (0.45f) /* 역제동 총 duty 상한 */
#define RETURN_STOP_POS       ((int32_t)(VCA_ADC_HOME + 800)) /* HOME+800 정지 판정 */
#define RETURN_CUSHION_DUTY   (0.12f)  /* 착륙 직후 쿠션 초기 duty */
#define RETURN_CUSHION_MS     (120U)   /* 쿠션 유지 시간(ms) */

#define VCA_STABLE_MS         (60U)
#define VCA_PID_DT_MS         (HP_SAMPLE_MS)
#define VCA_PID_DT_S          (0.001f)
#define DUTY_MOVE_MAX         (0.560f) /* 15V 기준 (0.35×1.6) */
#define DUTY_HOLD_MAX         (0.160f) /* 15V 기준 (0.10×1.6) */
#define HOLD_COAST_ERR        (600)
#define DUTY_HOLD_MAX_PUSH    (DUTY_HOLD_MAX)
#define DUTY_HOLD_MAX_PULL    (0.02f)
#define VCA_ADC_AVG_N         (4U)
#define REV_INHIBIT_ERR       (800)
#define DUTY_SLEW_STEP        (0.01f)
#define HOLD_UPDATE_MS        (50U)
#define DIR_MIN_SWITCH_MS     (60U)
#define QUIET_SAMPLE_MS       (0U)
#define VCA_VEL_DAMP_K        (0.00015f)
#define HOLD_KI_HZ            (0.00003f)
#define HOLD_ERR_I_ENABLE     (2500)
#define HOLD_VEL_I_ENABLE     (60)
#define HOLD_VEL_DEADBAND     (40)
#define HOLD_I_LEAK           (0.985f)
#define HOLD_BIAS_ENABLE_ERR  (1000)
#define VCA_VEL_DAMP_ALPHA    (0.2f)
#define DUTY_ZERO_EPS         (0.01f)
#define DUTY_HARD_MAX         (1.0f)

#define VCA_ADC_SAT_THRESHOLD (60000)
#define VCA_ADC_EXIT_TARGET   (100)
#define VCA_INIT_PUSH_DUTY    (0.20f)
#define VCA_INIT_TIMEOUT_MS   (500)

#define PRESS_DB_MS           (4U)
#define RELEASE_DB_MS         (120U)
#define RELEASE_GRACE_MS      (400U)
#define HOLD_ENTRY_BLOCK_MS   (200U)

/* ═══════════════════════════════════════════════════════════════════════════
 *  유틸리티 함수
 * ═══════════════════════════════════════════════════════════════════════════*/

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline void PID_Reset(PID_t *p)
{
    p->i_acc   = 0.0f;
    p->prev_e  = 0.0f;
}

static inline float PID_Update(PID_t *p, float setpoint, float meas, float dt_s)
{
    float e  = setpoint - meas;
    float up = p->kp * e;
    p->i_acc += (p->ki * e * dt_s);
    float ud  = p->kd * ((e - p->prev_e) / dt_s);
    p->prev_e = e;
    float u   = up + p->i_acc + ud;
    if      (u > p->out_max) { u = p->out_max; if (p->i_acc > p->out_max) p->i_acc = p->out_max; }
    else if (u < p->out_min) { u = p->out_min; if (p->i_acc < p->out_min) p->i_acc = p->out_min; }
    return u;
}

static inline bool in_deadband_i32(int32_t e, int32_t db)
{
    return (e <= db) && (e >= -db);
}

static inline int32_t iabs32(int32_t x) { return (x < 0) ? -x : x; }

static uint16_t median5_u16(const uint16_t h[5])
{
    uint16_t v[5] = {h[0], h[1], h[2], h[3], h[4]};
    for (int i = 0; i < 5; i++)
        for (int j = i + 1; j < 5; j++)
            if (v[j] < v[i]) { uint16_t t = v[i]; v[i] = v[j]; v[j] = t; }
    return v[2];
}

static inline uint16_t VCA_ReadPosADC(uint8_t avgN)
{
    (void)avgN;
    return g_vca_pos_adc;   /* ADS8325_AcqTask가 1ms마다 업데이트 */
}

/* ─── TLE9201 래퍼 (Loop-2 에서 방향만 제어, duty 는 Loop-1 이 담당) ────
 *  Loop-1 이 활성화된 동안에는 VCA_SetDuty() 를 Loop-2 가 직접
 *  호출하지 않습니다.  단, 방향(DIR)과 Enable 은 Loop-2 가 제어합니다.
 * ─────────────────────────────────────────────────────────────────────────*/

/* TLE9201 24bit 에러 클리어 – tle9201.c의 ClearErrors()는 16bit로 잘못됨
 * 실제 TLE9201: WR_DIAG(Clear) = 0x87 0x00 0x00 (24bit SPI)             */
static void TLE9201_ClearErrors24(void)
{
    extern SPI_HandleTypeDef hspi6;
    const uint8_t tx[3] = {0x87, 0x00, 0x00};
    uint8_t rx[3];
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi6, (uint8_t*)tx, rx, 3, 10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    osDelay(1);
    const uint8_t nop[3] = {0x00, 0x00, 0x00};
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi6, (uint8_t*)nop, rx, 3, 10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
}

/* ★ TIM8 CCR1 직접 주소 쓰기 (offset 0x34 실측 확인)
 * VCA_SetDuty()의 __HAL_TIM_SET_COMPARE가 shadow register 문제로
 * 즉시 반영 안 되는 현상 우회                                             */
static inline void VCA_SetDuty(float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    uint32_t arr = TIM8->ARR;
    uint32_t ccr = (uint32_t)((float)(arr + 1U) * duty);
    /* offset 0x34 = CCR1 실측 확인 */
    *((volatile uint32_t*)(0x40013400UL + 0x34UL)) = ccr;
    /* UEV 금지: AutomaticOutput=ENABLE 환경에서 소프트웨어 UEV가 MOE를 리셋할 수 있음 */
    /* AutoReloadPreload=DISABLE이므로 CCR도 즉시 반영됨 - UEV 불필요 */
    TIM8->BDTR |= TIM_BDTR_MOE;  /* MOE 항상 확인 */
}

static inline void vca_on(void)
{
    HAL_GPIO_WritePin(TLE9201_DIS_GPIO_Port, TLE9201_DIS_Pin, GPIO_PIN_RESET);
}

static inline void vca_off(void)
{
    Loop1_Stop();
    VCA_SetDuty(0.0f);
    HAL_GPIO_WritePin(TLE9201_DIS_GPIO_Port, TLE9201_DIS_Pin, GPIO_PIN_SET);
}

/* ─── vca_apply_u : Loop-2 에서 방향/enable 제어 + Loop-1 전류 지령 ──────
 *  u > 0  → PUSH 방향, |u| 를 전류(mA)로 환산하여 Loop-1 에 전달
 *  u < 0  → PULL 방향
 * ─────────────────────────────────────────────────────────────────────────*/
static inline void vca_apply_u(float u, int32_t err)
{
    static bool    last_dir_push       = true;
    static float   duty_prev           = 0.0f;
    static uint32_t last_dir_change_ms = 0;
    (void)duty_prev;  /* Loop-1 이 duty 관리 */

    float au = fabsf(u);

    if (au < DUTY_ZERO_EPS) {
        /* 거의 0 → 전류 지령 최솟값 유지 (완전 OFF 는 vca_off) */
        Loop1_SetTarget(0);
        g_u_dbg    = u;
        g_duty_dbg = 0.0f;
        g_dir_dbg  = last_dir_push ? 1 : 0;
        g_en_dbg   = 1;
        return;
    }

    bool want_push = (u >= 0.0f);
    uint32_t now_ms = osKernelGetTickCount();

    /* 방향 반전 억제 */
    if (want_push != last_dir_push) {
        if (iabs32(err) < REV_INHIBIT_ERR) {
            want_push = last_dir_push;
            u = last_dir_push ? +au : -au;
        } else if ((now_ms - last_dir_change_ms) < DIR_MIN_SWITCH_MS) {
            want_push = last_dir_push;
            u = last_dir_push ? +fabsf(u) : -fabsf(u);
        } else {
            last_dir_change_ms = now_ms;
        }
    }

    /* 방향 설정 (Loop-2 담당) */
    last_dir_push = want_push;
    TLE9201_SetDir(want_push ? DIR_PUSH : DIR_PULL);

    /* 전류 지령 (duty → mA 변환: I_CMD_MOVE_MA 를 full scale 로 매핑) */
    int32_t i_cmd = (int32_t)(fabsf(u) * (float)I_CMD_MOVE_MA);
    if (!want_push) i_cmd = -i_cmd;

    Loop1_SetTarget(i_cmd);

    g_u_dbg    = u;
    g_duty_dbg = s_duty_l1;
    g_dir_dbg  = want_push ? 1 : 0;
    g_en_dbg   = 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  상태 머신 열거형
 * ═══════════════════════════════════════════════════════════════════════════*/
typedef enum {
    VCA_HOME_OFF = 0,
    VCA_PROBE_LOAD,        /* 1a: ~1mm PUSH + 전류 측정 → 부하 판별 */
    VCA_APPROACH_TARGET,   /* 미사용 (호환 유지) */
    VCA_MOVE_TO_TARGET,    /* 1b: Loop1 전류제어 PUSH → 목표까지 */
    VCA_HOLD_TARGET,       /* 2 : Loop2 duty 직접제어 HOLD */
    VCA_RETURN_HOME        /* 3 : PULL → 부드러운 착륙 */
} VCA_State_t;

/* ─── Startup 래퍼 ────────────────────────────────────────────────────────*/
static inline void VCA_StartupPushWrapToZeroOnce(void)
{
    uint16_t p_prev = VCA_ReadPosADC(8);
    if (p_prev <= VCA_WRAP_LOW_TH) return;
    HAL_GPIO_WritePin(TLE9201_DIS_GPIO_Port, TLE9201_DIS_Pin, GPIO_PIN_RESET);
    TLE9201_SetDir(DIR_PUSH);
    const uint32_t on_ms = 12U, off_ms = 18U, max_pulses = 60U;
    const uint32_t t0 = HAL_GetTick();
    for (uint32_t i = 0; i < max_pulses; i++) {
        VCA_SetDuty(0.35f);  osDelay(on_ms);
        VCA_SetDuty(0.0f);   osDelay(off_ms);
        uint16_t p = VCA_ReadPosADC(8);
        if ((p_prev >= VCA_WRAP_HIGH_TH) && (p <= VCA_WRAP_LOW_TH)) break;
        if (p <= VCA_WRAP_TARGET) break;
        if ((HAL_GetTick() - t0) > 1200U) break;
        p_prev = p;
    }
    VCA_SetDuty(0.0f);
    HAL_GPIO_WritePin(TLE9201_DIS_GPIO_Port, TLE9201_DIS_Pin, GPIO_PIN_SET); VCA_SetDuty(0.0f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  HPSwitchTask  (Loop-2, 1 ms FreeRTOS)
 *
 *  변경점:
 *   - 상태 전환 시 Loop1_Start() / Loop1_SetTarget() 호출
 *   - APPROACH: 부스트 모드로 Loop1 시작 (피부 관통력 확보)
 *   - HOLD    : 저전류로 전환 (발열 최소화)
 *   - RETURN  : PULL 전류 지령
 *   - TLE9201 SetPWM_duty 는 Loop-1 이 관리하므로 직접 호출 금지
 *     (단, vca_on/vca_off 에서는 초기화 목적으로 0 설정 허용)
 * ═══════════════════════════════════════════════════════════════════════════*/
void HPSwitchTask(void *argument)
{
    static float   s_hold_base_duty = 0.0f;  (void)s_hold_base_duty;
    static uint8_t s_hold_latched   = 0U;    (void)s_hold_latched;
    (void)argument;

    /* ★ 부팅 시 TLE9201 강제 OFF (IOC 초기값과 무관하게 확실히 비활성화) */
    HAL_GPIO_WritePin(TLE9201_DIS_GPIO_Port, TLE9201_DIS_Pin, GPIO_PIN_SET); /* DIS=HIGH=OFF */
    VCA_SetDuty(0.0f);
    osDelay(50);

    VCA_Init();   /* 내부에서 TLE9201_SetPWM_Frequency(40000) 호출 */
    /* 주파수를 20kHz로 낮춤: 같은 duty에서 더 강한 전류 */
    extern void TLE9201_SetPWM_Frequency_pub(uint32_t);  /* tle9201.c에 없으면 직접 */
    __HAL_TIM_DISABLE(&htim8);
    __HAL_TIM_SET_PRESCALER(&htim8, 0);
    /* 20kHz: ARR = 50MHz/20kHz - 1 = 2499 */
    TIM8->ARR = 2499U;
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    __HAL_TIM_ENABLE(&htim8);
    HAL_GPIO_WritePin(TLE9201_DIS_GPIO_Port, TLE9201_DIS_Pin, GPIO_PIN_SET);
    VCA_SetDuty(0.0f);

    /* CC1NE(complementary) 비활성화 - CH1만 사용 */
    TIM8->CCER &= ~TIM_CCER_CC1NE;
    TIM8->CCER |=  TIM_CCER_CC1E;
    TIM8->CCMR1 = (TIM8->CCMR1 & ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC1PE))
                | (0x6U << TIM_CCMR1_OC1M_Pos);
    TIM8->BDTR  |= TIM_BDTR_MOE;
    *((volatile uint32_t*)(0x40013400UL + 0x34UL)) = 0;  /* CCR1=0 직접 */
    osDelay(10);

    /* ─── 스위치 디바운스 변수 ─── */
    uint8_t raw_now = (HAL_GPIO_ReadPin(HP_SW_GPIO_Port, HP_SW_Pin) == GPIO_PIN_SET) ? 0U : 1U;
    static bool     s_latched_push      = false;
    static uint32_t s_press_cnt         = 0;
    static uint32_t s_release_cnt       = 0;
    static uint32_t s_release_block_until = 0;
    static bool     s_sw_init           = false;
    if (!s_sw_init) { s_latched_push = (raw_now == 1U); s_sw_init = true; }

    bool last_in_push = s_latched_push;
    uint32_t stable_t0   = 0;
    static uint32_t s_push_t0   = 0;  (void)s_push_t0;
    static uint16_t s_push_pos0 = 0;  (void)s_push_pos0;
    static uint32_t release_t0  = 0;  /* 루프 밖: edge·gate 공유 */
    static int32_t  s_push_i_ma = 0;  /* PROBE 결과로 결정된 PUSH 전류 */
    static uint32_t s_probe_t0  = 0;  /* PROBE 타임아웃 기준 */
    static uint32_t s_move_t0   = 0;  /* MOVE 타임아웃 기준 */
    static uint16_t s_adc_at_home = 0; /* PROBE 진입 직전 실제 home ADC */
    static int32_t  s_pos_prev    = 0;
    static float    s_vel_lp      = 0.0f;
    static float    s_loaded_duty = 0.0f;  /* 유부하 Loop2 전류제어 duty 누적 */

    VCA_State_t s_state = last_in_push ? VCA_MOVE_TO_TARGET : VCA_HOME_OFF;

    static PID_t s_pid_move = {
        .kp = 0.000005f, .ki = 0.0f, .kd = 0.0f, .dt = 0.001f,
        .out_min = -DUTY_MOVE_MAX, .out_max = +DUTY_MOVE_MAX,
        .i_acc = 0.0f, .prev_e = 0.0f
    };
    static PID_t s_pid_hold = {
        .kp = 0.00016f, .ki = (HOLD_KI_HZ / 2.0f), .kd = 0.0f, .dt = 0.001f,
        .out_min = -DUTY_HOLD_MAX, .out_max = +DUTY_HOLD_MAX,
        .i_acc = 0.0f, .prev_e = 0.0f
    };
    PID_Reset(&s_pid_move);
    PID_Reset(&s_pid_hold);

    if (last_in_push) {
        vca_on();
        TLE9201_SetDir(DIR_PUSH);
        {   /* PROBE duty = min(PROBE_DUTY, 선형화 duty) */
            float pd = depth_to_duty(g_needle_depth_mm);
            if (pd > PROBE_DUTY) pd = PROBE_DUTY;
            VCA_SetDuty(pd);
        }
        s_push_i_ma = PUSH_I_NO_LOAD;
        s_probe_t0  = osKernelGetTickCount();
        s_adc_at_home = g_vca_pos_adc;  /* 모터 시작 전 실제 home ADC 캡처 */
        g_motor_active = 1;  /* PROBE 시작 → ADC 동결 (EMI 방지) */
        s_state = VCA_PROBE_LOAD;
    } else {
        vca_off();
        s_state = VCA_HOME_OFF;
    }

    /* ═══════════════════════ MAIN LOOP ═══════════════════════════════════ */
    for (;;)
    {
        static int s_prev_state = -1;
        if ((int)s_state != s_prev_state) {
            if (s_prev_state == (int)VCA_HOLD_TARGET &&
                (int)s_state  != (int)VCA_HOLD_TARGET) {
                s_hold_latched = 0U;
            }
            s_prev_state = (int)s_state;
        }
        uint32_t now = osKernelGetTickCount();

        /* ── 0. ADS7041 전류 읽기 (태스크에서 SPI 안전 호출) ──────────── */
        {
            uint16_t raw = ADS7041_ReadRaw();
            g_adc7041_raw = raw;
            g_i_meas_mA   = (int32_t)(((int32_t)raw - ADC7041_BIAS_CODE)
                                       * ADC7041_MA_PER_LSB_F);
            g_i_meas_updated = 1;  /* ISR에 새 값 도착 알림 */
        }

        /* ── 1. 스위치 디바운스 ────────────────────────────────────────── */
        /* PA8 PULLUP: 누르면 LOW(0V) → pressed=1, 놓으면 HIGH → pressed=0 */
        uint8_t raw = (HAL_GPIO_ReadPin(HP_SW_GPIO_Port, HP_SW_Pin) == GPIO_PIN_SET) ? 0U : 1U;
        if (!s_latched_push) {
            if (raw == 1U) {   /* 눌림 */
                if (++s_press_cnt >= (PRESS_DB_MS / HP_SAMPLE_MS)) {
                    s_latched_push  = true;
                    s_press_cnt     = 0;
                    s_release_cnt   = 0;
                    s_release_block_until = now + RELEASE_GRACE_MS;
                }
            } else s_press_cnt = 0;
        } else {
            if (raw == 0U) {   /* 놓음 */
                if (now < s_release_block_until) {
                    s_release_cnt = 0;
                } else {
                    if (++s_release_cnt >= (RELEASE_DB_MS / HP_SAMPLE_MS)) {
                        s_latched_push  = false;
                        s_release_cnt   = 0;
                        s_press_cnt     = 0;
                    }
                }
            } else s_release_cnt = 0;
        }
        bool in_push = s_latched_push;
        g_sw_state = in_push ? 1U : 0U;
        g_blk_remain = (int32_t)s_release_block_until - (int32_t)now;

        /* ── 2. 위치 필터링 ────────────────────────────────────────────── */
        uint16_t pos_raw = VCA_ReadPosADC(VCA_ADC_AVG_N);
        static uint16_t s_hist[5]   = {0};
        static uint8_t  s_hi        = 0;
        static bool     s_hist_init = false;
        static float    s_pos_lp    = 0.0f;
        if (!s_hist_init) {
            for (int i = 0; i < 5; i++) s_hist[i] = pos_raw;
            s_pos_lp   = (float)pos_raw;
            s_hist_init = true;
        } else {
            s_hist[s_hi] = pos_raw;
            s_hi = (uint8_t)((s_hi + 1U) % 5U);
        }
        uint16_t pos_med = median5_u16(s_hist);
        const float alpha = 0.12f;
        s_pos_lp = s_pos_lp + alpha * ((float)pos_med - s_pos_lp);
        uint16_t pos = (uint16_t)(s_pos_lp + 0.5f);
        g_sm_pos = pos;   /* UART 모니터링용 */

        /* ── 2-b. 하드 리밋 제거 ─────────────────────────────────────────
         *  기존 38000 하드 리밋은 오버슈트 시 duty=0 → 전체 낙하 → 재push
         *  → 진동 유발. MOVE/HOLD 내 VCA_ADC_MAX_SAFE(39000) 체크로 충분 */

        /* 속도 추정 */
        int32_t dpos  = (int32_t)pos - s_pos_prev;
        s_pos_prev    = (int32_t)pos;
        s_vel_lp      = s_vel_lp + VCA_VEL_DAMP_ALPHA * ((float)dpos - s_vel_lp);

        /* ── 3. 스위치 에지 이벤트 ─────────────────────────────────────── */
        if (in_push != last_in_push) {
            last_in_push = in_push;
            if (in_push) {
                /* 스위치 ON: PROBE 상태로 진입 (저duty로 1mm 전진하며 전류 측정) */
                s_push_t0   = now;
                s_push_pos0 = (uint16_t)pos;
                PID_Reset(&s_pid_move);
                PID_Reset(&s_pid_hold);
                g_load_detected = 0U;
                s_push_i_ma = PUSH_I_NO_LOAD;
                vca_on();
                TLE9201_SetDir(DIR_PUSH);
                {   float pd = depth_to_duty(g_needle_depth_mm);
                    if (pd > PROBE_DUTY) pd = PROBE_DUTY;
                    VCA_SetDuty(pd);
                }
                s_probe_t0 = now;
                s_adc_at_home = g_vca_pos_adc;  /* home ADC 캡처 */
                g_motor_active = 1;  /* PROBE 재진입 → ADC 동결 */
                s_state    = VCA_PROBE_LOAD;
                s_release_block_until = now + RELEASE_GRACE_MS;
                release_t0 = 0U;
                stable_t0  = 0;
            }
            /* PULL(스위치 OFF)은 아래 Release Gate 처리 */
        }

        /* ── 4. Release Gate ────────────────────────────────────────────── */
        float u = 0.0f;  (void)u;

        if (!in_push && s_state == VCA_HOME_OFF) {
            osDelay(VCA_PID_DT_MS);
            continue;
        }

        if (!in_push) {
            /* 스위치 OFF: Loop1 즉시 차단, RETURN 상태로 전환 */
            if (release_t0 == 0U) {
                release_t0 = now;
                Loop1_Stop();
                VCA_SetDuty(0.0f);
                g_motor_active = 0;  /* 모터 정지 → ADC 동결 해제 */
                vca_on();  /* 하드 리밋이 DIS=HIGH 했을 수 있으므로 재활성화 */
                PID_Reset(&s_pid_move);
                PID_Reset(&s_pid_hold);
                s_state = VCA_RETURN_HOME;
            }
            /* HOME 착륙 완료(RETURN 상태에서만) 또는 타임아웃 1.5s */
            bool pos_at_home = (s_state == VCA_RETURN_HOME) &&
                                ((int32_t)pos <= (int32_t)RETURN_STOP_POS);
            /* 위치 도달 + 속도 거의 0 + 쿠션 시간 경과 후에만 OFF */
            bool vel_settled = (s_vel_lp > -2.0f && s_vel_lp < 2.0f);
            static uint32_t s_home_arrive_t = 0;
            if (pos_at_home && vel_settled) {
                if (s_home_arrive_t == 0) s_home_arrive_t = now;
            } else {
                s_home_arrive_t = 0;
            }
            bool home_reached = (s_home_arrive_t != 0) &&
                                ((now - s_home_arrive_t) >= RETURN_CUSHION_MS);
            bool timeout = ((now - release_t0) >= 1500U);
            if (home_reached || timeout) {
                VCA_SetDuty(0.0f);
                vca_off();
                s_state    = VCA_HOME_OFF;
                release_t0 = 0U;
                s_home_arrive_t = 0;
            }
            osDelay(VCA_PID_DT_MS);
            continue;
        } else {
            release_t0 = 0U;
        }

        /* ── 5. 상태 머신 ──────────────────────────────────────────────── */
        switch (s_state)
        {

        /* ══ 1a. PROBE: 부하 판별 ════════════════════════════════════════
         *  PROBE_DUTY 로 천천히 PUSH하면서 g_i_meas_mA 를 읽어
         *  PROBE_CURRENT_TH 이상이면 피부 있음(유부하) 판정.
         *  PROBE_ADC_END(1mm) 도달 또는 타임아웃 시 MOVE로 전환.
         *
         *  ★ PROBE_CURRENT_TH 튜닝 ★
         *  무부하 상태에서 전류값을 읽어 확인 후, 그 값의 2~3배로 설정.
         * ═══════════════════════════════════════════════════════════════*/
        case VCA_PROBE_LOAD: {
            g_state_dbg = 1;
            TLE9201_SetDir(DIR_PUSH);

            /* PROBE duty = min(PROBE_DUTY, 선형화 duty)
             *  얕은 목표: 선형화 duty < PROBE_DUTY → 선형화 duty 사용
             *  깊은 목표: 선형화 duty > PROBE_DUTY → PROBE_DUTY 사용
             *  → 오버슈트 방지: PROBE가 HOLD보다 강하게 밀지 않음 */
            {
                float pd = depth_to_duty(g_needle_depth_mm);
                if (pd > PROBE_DUTY) pd = PROBE_DUTY;
                VCA_SetDuty(pd);
            }
            g_motor_active = 1;  /* PROBE 중 ADC 동결 유지 */

            /* 전류 측정으로 부하 판별 (전류 ADC는 EMI 영향 적음) */
            int32_t i_now = g_i_meas_mA;
            if (i_now > (int32_t)PROBE_CURRENT_TH) {
                /* 유부하: 피부 감지 → 높은 전류로 관통 */
                g_load_detected = 1U;
                s_push_i_ma = PUSH_I_LOADED;
            }

            /* PROBE 완료: 목표 깊이에 비례한 시간 (최소 100ms 부하감지)
             *  G10(얕음): 100ms → 과도 push 방지
             *  G30(깊음): ~225ms → 충분한 이동 */
            #define PROBE_MIN_MS  50U
            uint32_t probe_limit;
            {
                int32_t tgt = DEPTH_MM_TO_ADC(g_needle_depth_mm);
                if (tgt < (int32_t)VCA_ADC_HOME) tgt = (int32_t)VCA_ADC_HOME;
                probe_limit = (uint32_t)(
                    (uint32_t)(tgt - (int32_t)VCA_ADC_HOME) * PROBE_TIMEOUT_MS
                    / (uint32_t)(VCA_ADC_MAX_SAFE - VCA_ADC_HOME));
                if (probe_limit < PROBE_MIN_MS) probe_limit = PROBE_MIN_MS;
                if (probe_limit > PROBE_TIMEOUT_MS) probe_limit = PROBE_TIMEOUT_MS;
            }
            bool probe_done = ((now - s_probe_t0) >= probe_limit);
            if (probe_done) {
                /* PROBE 끝 → MOVE로 전환
                 * 유부하/무부하 모두 Loop2 duty 직접제어 (Loop1 발진 문제) */
                if (g_load_detected != 0U) {
                    Loop1_Stop();
                    s_loaded_duty = PROBE_DUTY;  /* PROBE duty에서 연속 시작 */
                    TLE9201_SetDir(DIR_PUSH);
                } else {
                    Loop1_Stop();
                    TLE9201_SetDir(DIR_PUSH);
                    VCA_SetDuty(NO_LOAD_MOVE_DUTY);
                }
                s_state = VCA_MOVE_TO_TARGET;
                stable_t0 = 0;
                s_move_t0 = now;     /* MOVE 타이머 시작 */
                g_motor_active = 1;  /* MOVE 진입 → ADC 동결 */
            }
            break;
        }

        /* ══ 1b. MOVE: Loop1 전류제어로 목표까지 PUSH ════════════════════
         *  3단계 제동:
         *  [HOME+1mm] ─ PUSH_I(결정된값) ─ [목표-BRAKE_ZONE] ─ PUSH_I_BRAKE ─ [목표-DB] ─ HOLD
         * ═══════════════════════════════════════════════════════════════*/
        case VCA_APPROACH_TARGET:   /* 호환 유지 — MOVE로 즉시 */
            Loop1_Start(s_push_i_ma, (g_load_detected != 0U));
            s_state = VCA_MOVE_TO_TARGET;
            break;

        case VCA_MOVE_TO_TARGET: {
            g_state_dbg = 2;

            /* ── MOVE = HOLD FF duty로 밀기 ──────────────────────────
             *  HOLD과 동일한 duty 사용 → 모터-스프링 평형점 = 목표위치
             *  오버슈트 원천 차단: push force = hold force
             *  800ms 후 HOLD 전환 (VCA가 평형점에 도달할 시간)
             * ──────────────────────────────────────────────────────── */
            #define MOVE_SETTLE_MS  800U

            float depth = g_needle_depth_mm;
            if (depth < 0.0f) depth = 0.0f;
            float move_duty = depth_to_duty(depth);
            if (move_duty > HOLD_DUTY_MAX) move_duty = HOLD_DUTY_MAX;

            TLE9201_SetDir(DIR_PUSH);
            VCA_SetDuty(move_duty);
            g_motor_active = 1;

            /* 그래프: 실측 home + depth*PER_MM = 추정 위치 표시 */
            {
                int32_t est = (int32_t)s_adc_at_home
                            + (int32_t)(depth * (float)VCA_ADC_PER_MM);
                if (est > 65535) est = 65535;
                g_vca_pos_adc = (uint16_t)est;
            }

            /* 800ms 후 HOLD 전환 */
            if (s_move_t0 != 0 && (now - s_move_t0) >= MOVE_SETTLE_MS) {
                Loop1_Stop();
                s_state   = VCA_HOLD_TARGET;
                stable_t0 = 0;
                break;
            }

            g_duty_dbg = move_duty;
            g_dir_dbg  = 1;
            break;
        }

        /* ══ 2. HOLD: 깊이 비례 feedforward, 모터 계속 ON ═══════════════
         *  EMI 때문에 모터 ON 중 ADC 읽기 불가.
         *  모터 OFF 하면 스프링이 VCA를 끌어당겨 진동.
         *  → 해법: ADC 읽기를 포기하고, 깊이에 비례하는 duty만 적용
         *  duty = depth_to_duty(depth_mm)  — 선형화 LUT 보간
         *  비선형 VCA 특성을 보상하여 원하는 깊이 정밀 유지
         * ═══════════════════════════════════════════════════════════════*/
        case VCA_HOLD_TARGET: {
            g_state_dbg = 3;

            /* ── 깊이(mm) → hold duty 변환 (선형화 LUT) ──
             *  측정 데이터 기반 구간별 선형 보간
             *  최대 duty 0.60 */
            float depth = g_needle_depth_mm;
            if (depth < 0.0f) depth = 0.0f;

            float duty_hold = depth_to_duty(depth);
            if (duty_hold > HOLD_DUTY_MAX) duty_hold = HOLD_DUTY_MAX;

            TLE9201_SetDir(DIR_PUSH);
            VCA_SetDuty(duty_hold);
            g_motor_active = 1;   /* ADC 동결 유지 — EMI 완전 차단 */

            /* 그래프: 실측 home + depth*PER_MM = 추정 위치 표시 */
            {
                int32_t est = (int32_t)s_adc_at_home
                            + (int32_t)(depth * (float)VCA_ADC_PER_MM);
                if (est > 65535) est = 65535;
                g_vca_pos_adc = (uint16_t)est;
            }
            g_vca_err  = 0;  /* feedforward 전용 → err 무의미 */
            g_duty_dbg = duty_hold;
            g_en_dbg   = 1;
            break;
        }

        /* ══ 3. RETURN: 스프링 복귀 + PUSH 역제동 부드러운 착륙 ══════════
         *  핵심: HOME 근처에서 PULL 하지 않는다!
         *  - 먼 거리: 약한 PULL로 스프링 보조
         *  - 중간: duty=0, 스프링 복원력만으로 복귀 (coast)
         *  - HOME 가까이: PUSH 역제동 (거리+속도 비례)
         *  - 정지 후: 쿠션 duty를 서서히 줄여 충돌 방지
         * ═══════════════════════════════════════════════════════════════*/
        case VCA_RETURN_HOME: {
            g_state_dbg = 4;
            if (s_l1_enable) Loop1_Stop();

            int32_t dist_to_home = (int32_t)pos - (int32_t)VCA_ADC_HOME;
            float vel = s_vel_lp;  /* 음수 = HOME 방향 하강 */

            if ((int32_t)pos <= (int32_t)RETURN_STOP_POS) {
                /* ── 착륙: 쿠션 PUSH로 잔여 에너지 흡수 ────────────
                 * 쿠션 duty를 시간에 따라 선형 감소 → 부드럽게 0 */
                static uint32_t s_cushion_t0 = 0;
                if (s_cushion_t0 == 0) s_cushion_t0 = now;

                uint32_t elapsed = now - s_cushion_t0;
                if (elapsed < RETURN_CUSHION_MS) {
                    /* 시간 비례 감소 + 속도 비례 보조 */
                    float time_fade = 1.0f - (float)elapsed / (float)RETURN_CUSHION_MS;
                    float cushion = RETURN_CUSHION_DUTY * time_fade;
                    /* 아직 하강 중이면 속도 비례 보조 추가 */
                    if (vel < -1.0f) cushion += (-vel * 0.0010f);
                    if (cushion > 0.35f) cushion = 0.35f;
                    if (cushion < 0.0f)  cushion = 0.0f;
                    TLE9201_SetDir(DIR_PUSH);
                    VCA_SetDuty(cushion);
                } else {
                    VCA_SetDuty(0.0f);
                    s_cushion_t0 = 0;
                }
            } else if (dist_to_home <= (int32_t)RETURN_BRAKE_ZONE) {
                /* ── PUSH 역제동 구간: 거리+속도 비례 ──────────────
                 * 가까울수록 강하게, 빠를수록 강하게 제동
                 * → HOME 도달 시 거의 속도 0 */
                /* 위치 비례: dist=0에서 최대, dist=BRAKE_ZONE에서 0 */
                float pos_ratio = 1.0f - (float)dist_to_home / (float)RETURN_BRAKE_ZONE;
                float pos_brake = pos_ratio * pos_ratio * RETURN_BRAKE_POS_MAX;

                /* 속도 비례: 하강 속도가 클수록 강한 제동 */
                float vel_brake = (vel < 0.0f) ? (-vel * RETURN_BRAKE_VEL_K) : 0.0f;

                float brake = pos_brake + vel_brake;
                if (brake > RETURN_BRAKE_TOTAL_MAX) brake = RETURN_BRAKE_TOTAL_MAX;
                if (brake < 0.02f) brake = 0.02f;  /* 최소 제동력 */

                TLE9201_SetDir(DIR_PUSH);
                VCA_SetDuty(brake);
            } else if (dist_to_home > (int32_t)RETURN_COAST_ZONE) {
                /* ── 먼 거리: 약한 PULL (스프링 보조) ── */
                TLE9201_SetDir(DIR_PULL);
                VCA_SetDuty(RETURN_FAST_DUTY);
            } else {
                /* ── Coast 구간: duty=0, 스프링만으로 복귀 ──
                 * PULL 하지 않아야 HOME 가까이에서 속도가 낮음 */
                VCA_SetDuty(0.0f);
            }
            break;
        }

        case VCA_HOME_OFF:
        default:
            Loop1_Stop();
            vca_off();
            break;
        }

        osDelay(VCA_PID_DT_MS);
    }
}  /* end HPSwitchTask */

/* ═══════════════════════════════════════════════════════════════════════════
 *  ADS8325 타이머 구동 ADC
 *
 *  TIM7 (4 kHz ISR) → 카운터 증가만 (~100ns)
 *  ADS8325_AcqTask (5 ms) → 카운터만큼 SPI burst → trimmed mean → IIR
 *
 *  SPI1: 2.667 MHz SCLK (CubeMX prescaler=16, 런타임 변경 금지)
 *  TIM7: 250 μs 주기 (4 kHz), 5ms에 ~20개 burst → 강력한 잡음 제거
 *  CPU: 20 × 14μs = 280μs / 5ms ≈ 5.6%
 * ═══════════════════════════════════════════════════════════════════════════*/

/* ── TIM7 ISR: 카운터만 증가 (SPI 접근 금지) ──
 *  HAL SPI는 내부 Lock+timeout → ISR에서 데드락 위험
 *  ISR: ~100ns (카운터 증가만)
 *  Task: 카운터만큼 SPI burst 읽기 → 안전 */
static volatile uint16_t s_adc_req = 0;  /* ISR이 증가, Task가 소비 */

void ADS8325_TIM7_ISR_Handler(void)
{
    if (!g_motor_active) {
        s_adc_req++;
    }
}

/* ── ADC Task: TIM7 카운터 기반 burst 읽기 + 필터 ── */
void ADS8325_AcqTask(void *argument)
{
    (void)argument;
    ADS8325_Init(&g_ads8325, &hspi1, GPIOA, GPIO_PIN_4);

    /* ⚠ SPI1 속도는 CubeMX에서 설정 (런타임 변경 금지)
     *  prescaler=16 (2.667MHz) → CubeMX에서 복원 필요
     *  런타임 HAL_SPI_Init/레지스터 수정은 UART 깨짐 부작용 */

    /* TIM7: 4 kHz 타이머 → 5ms마다 ~20개 burst
     *  ISR: 카운터 증가만 (~100ns)
     *  Task: 5ms 주기, ~20개 burst × 14μs = 280μs → CPU ~5.6%
     *  이전(9개/5ms, median+IIR0.15) → 개선(20개/5ms, trimmed mean+IIR0.10) */
    extern TIM_HandleTypeDef htim7;
    __HAL_TIM_SET_AUTORELOAD(&htim7, 15999U); /* 64MHz/(15999+1)=4kHz */
    HAL_TIM_Base_Start_IT(&htim7);

    /* ── 처리 파라미터 ── */
    #define BATCH_MAX     24    /* 한 번에 최대 처리 개수 (5ms×4kHz=20, 여유+4) */
    #define TRIM_PCT      25    /* 상하 25% 절삭 → 20개 중 10개만 평균 */
    #define IIR_ALPHA     (0.10f)  /* 강한 평활: τ≈50ms, 이전 0.15보다 강력 */
    #define SPIKE_TH      (3000)
    #define SETTLE_BATCHES 4    /* 모터 정지 후 20ms(4배치×5ms) 대기 */

    static uint8_t  s_adc_init   = 0;
    static uint8_t  s_settle_cnt = 0;
    static float    s_iir_acc    = 0.0f;

    for (;;) {
        osDelay(5);  /* 5ms 주기 — TIM7(4kHz)×5ms=20샘플 축적 */

        /* 모터 동작 중 → 카운터 리셋, settle 예약 */
        if (g_motor_active) {
            s_adc_req = 0;
            s_settle_cnt = SETTLE_BATCHES;
            continue;
        }

        /* 모터 정지 직후 → back-EMF 대기 */
        if (s_settle_cnt > 0) {
            s_adc_req = 0;
            s_settle_cnt--;
            continue;
        }

        /* ── TIM7 카운터에서 읽을 개수 결정 ── */
        uint16_t n_req = s_adc_req;
        s_adc_req = 0;
        if (n_req == 0) continue;
        if (n_req > BATCH_MAX) n_req = BATCH_MAX;

        /* ── SPI burst 읽기 (Task 컨텍스트 — 안전) ── */
        uint16_t buf[BATCH_MAX];
        for (uint16_t i = 0; i < n_req; i++) {
            buf[i] = ADS8325_Read(&g_ads8325);
        }

        /* ── 정렬 (insertion sort, N≤128) ── */
        uint16_t avail = n_req;
        for (int i = 1; i < (int)avail; i++) {
            uint16_t key = buf[i];
            int j = i - 1;
            while (j >= 0 && buf[j] > key) {
                buf[j + 1] = buf[j];
                j--;
            }
            buf[j + 1] = key;
        }

        /* ── wrap 감지 ── */
        uint16_t smin = buf[0];
        uint16_t smax = buf[avail - 1];
        if (smax >= 60000U && (smax - smin) > 15000U) {
            g_vca_pos_raw = 65535U;
            if (s_adc_init) {
                s_iir_acc = 65535.0f;
                g_vca_pos_adc = 65535U;
            }
            continue;
        }

        /* ── trimmed mean: 상하 TRIM_PCT% 절삭 ── */
        uint16_t trim_n = (uint16_t)((uint32_t)avail * TRIM_PCT / 100U);
        uint16_t lo = trim_n;
        uint16_t hi = avail - trim_n;
        if (hi <= lo) { lo = 0; hi = avail; }  /* 안전장치 */

        uint32_t sum = 0;
        for (uint16_t i = lo; i < hi; i++) sum += buf[i];
        uint16_t trimmed = (uint16_t)(sum / (hi - lo));

        g_vca_pos_raw = trimmed;  /* 디버그: 필터 전 값 */

        /* ── IIR 저역통과 + 스파이크 제거 ── */
        if (!s_adc_init) {
            s_iir_acc = (float)trimmed;
            s_adc_init = 1;
        } else {
            float diff = (float)trimmed - s_iir_acc;
            if (diff < 0.0f) diff = -diff;

            if (diff > (float)SPIKE_TH) {
                /* 스파이크 → IIR 유지 */
            } else {
                s_iir_acc = s_iir_acc * (1.0f - IIR_ALPHA)
                          + (float)trimmed * IIR_ALPHA;
            }
        }

        /* IIR → uint16_t */
        float out = s_iir_acc;
        if (out < 0.0f) out = 0.0f;
        if (out > 65535.0f) out = 65535.0f;
        g_vca_pos_adc = (uint16_t)(out + 0.5f);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  ADS8325_RS485_Task  (UART 전송)
 * ═══════════════════════════════════════════════════════════════════════════*/
void ADS8325_RS485_Task(void *argument)
{
    (void)argument;
    gUart2TxTaskHandle = xTaskGetCurrentTaskHandle();
    const TickType_t PERIOD = pdMS_TO_TICKS(10);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, PERIOD);
        /* nano.specs 에서 %f 가 동작 안 할 수 있으므로 정수로 변환 */
        int32_t duty_pct = (int32_t)(g_duty_dbg * 1000.0f);
        (void)0;  /* TLE9201_ReadDiag 제거 - 내부에서 TLE9201_Enable(false) 호출하여 DIS=HIGH 됨 */
        /* TIM8 레지스터 상태 출력 */
        /* TIM8 레지스터 맵 덤프 (base=0x40013400)
         * offset 0x00=CR1, 0x04=CR2, 0x08=SMCR, 0x0C=DIER
         * offset 0x10=SR,  0x14=EGR, 0x18=CCMR1,0x1C=CCMR2
         * offset 0x20=CCER,0x24=CNT, 0x28=PSC,  0x2C=ARR
         * offset 0x30=RCR, 0x34=CCR1,0x38=CCR2, 0x3C=CCR3
         * offset 0x40=CCR4,0x44=BDTR                        */
        int len = 0;
        len += snprintf(g_ads8325_buf, sizeof(g_ads8325_buf),
                        "$%u E%"PRId32";\r\n",
                        (unsigned)g_vca_pos_adc,
                        (int32_t)g_vca_err);
        HAL_StatusTypeDef st;
        do {
            st = Uart2_Tx_IT((uint8_t*)g_ads8325_buf, (uint16_t)len);
            if (st == HAL_BUSY) osDelay(10);
        } while (st == HAL_BUSY);
    }
}

static HAL_StatusTypeDef Uart1_Tx_IT(uint8_t *pData, uint16_t size)
{
    HAL_StatusTypeDef st = HAL_UART_Transmit_IT(&huart1, pData, size);
    if (st != HAL_OK) return st;
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == 0) return HAL_TIMEOUT;
    return HAL_OK;
}

static HAL_StatusTypeDef Uart2_Tx_IT(uint8_t *pData, uint16_t size)
{
    HAL_StatusTypeDef st = HAL_UART_Transmit_IT(&huart2, pData, size);
    if (st != HAL_OK) return st;
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == 0) return HAL_TIMEOUT;
    return HAL_OK;
}
/* ═══════════════════════════════════════════════════════════════════════════
 *  LEDTask
 * ═══════════════════════════════════════════════════════════════════════════*/
void LEDTask(void *argument)
{
    (void)argument;
    for (;;) osDelay(1000);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SysMonTask
 * ═══════════════════════════════════════════════════════════════════════════*/
void SysMonTask(void *argument)
{
    (void)argument;
    const size_t    HEAP_WARN  = 2048;
    const size_t    HEAP_CRIT  = 1024;
    const uint32_t  PERIOD_MS  = 1000;
    for (;;) {
        size_t freeHeap    = xPortGetFreeHeapSize();
        size_t minEverHeap = xPortGetMinimumEverFreeHeapSize();
        size_t qFree  = UART_Cmd_GetRxQueueFree();
        size_t qTotal = UART_Cmd_GetRxQueueLength();
        if (minEverHeap < HEAP_CRIT) {
            char msg[128];
            snprintf(msg, sizeof(msg), "\r\n[ALARM] Heap CRITICAL! free=%u, minEver=%u\r\n",
                     (unsigned)freeHeap, (unsigned)minEverHeap);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
        } else if (minEverHeap < HEAP_WARN) {
            char msg[128];
            snprintf(msg, sizeof(msg), "\r\n[WARN] Heap low. free=%u, minEver=%u\r\n",
                     (unsigned)freeHeap, (unsigned)minEverHeap);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
        }
        static size_t   last_used  = 0;
        static uint32_t last_print = 0;
        if (qTotal > 0) {
            size_t used = qTotal - qFree;
            uint32_t t  = osKernelGetTickCount();
            if ((used * 4 >= qTotal * 3) && (t - last_print >= 5000) && (used != last_used)) {
                last_used  = used;
                last_print = t;
                char msg[128];
                snprintf(msg, sizeof(msg), "\r\n[WARN] UART RX queue high: used=%u / %u\r\n",
                         (unsigned)used, (unsigned)qTotal);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
            }
        }
        osDelay(PERIOD_MS);
    }
}
