#include "stm32h5xx_hal.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "usart.h"
#include "uart_cmd.h"
#include <ctype.h>

// VCA PID / Target 제어용 extern (my_tasks.c에서 구현)
extern void     VCA_PID_SetGains(float kp, float ki, float kd);
extern void     VCA_PID_GetGains(float *kp, float *ki, float *kd);
extern void     VCA_SetTargetADC(uint16_t adc_target);
extern uint16_t VCA_GetTargetADC(void);
extern osThreadId_t ads8325TaskHandle;

// 외부 UART 핸들 (명령 포트는 UART1)
extern UART_HandleTypeDef huart1;

// G 명령으로 저장해 둘 pending target (ADC 단위)
static uint16_t s_vca_pending_target_adc = 25000;

// V 명령용: 25k <-> 45k 1Hz 왕복
static bool     s_vca_vmode_enable = false;
static uint16_t s_vca_v_low  = 25000;
static uint16_t s_vca_v_high = 35000;
static uint32_t s_vca_v_last_tick = 0;
static uint32_t s_vca_v_half_period_ms = 500;   // ✅ 기본 1Hz (500ms)

// ================== 설정 ==================
#define CMD_LINE_MAX    64
#define RXQ_LEN         64
#define TASKSTAT_MAX    16

// ================== 정적 자원 ==================
static QueueHandle_t s_rxq = NULL;
static uint8_t       s_rx_byte;            // IT 재예약용 1바이트
static TaskHandle_t  sUartCmdTaskHandle = NULL;

// Task 통계용 전역 버퍼 (스택 절약)
static TaskStatus_t gTaskStatus[TASKSTAT_MAX];

// 't' 통계는 리셋 시 자동으로 1회 출력하고, 이후에도 1회 입력으로 바로 출력되게 구성

// ================== 로컬 함수 선언 ==================
static void Terminal_ClearScreenAndBanner(void);
static void uart_write_str(const char *s);
static void uart_write_len(const uint8_t *buf, size_t n);
static void uart_send_data(const uint8_t *buf, uint16_t len);
static void uart_echo_byte(uint8_t ch);

static void print_task_stats(UART_HandleTypeDef *huart);
static void print_heap_stats(void);
static void handle_user_command(const char *line);



// ---- CPU 런타임 카운터 기준값 (TC 명령용) ----
typedef struct {
    TaskHandle_t handle;
    uint32_t     baseCounter;
} TaskCpuBaseline_t;

static TaskCpuBaseline_t gTaskCpuBaseline[TASKSTAT_MAX];
static configRUN_TIME_COUNTER_TYPE gBaseTotalRunTime = 0;
static uint8_t  gBaseValid = 0;

static void TaskCpu_ResetAllBaselines(void);
static uint32_t TaskCpu_GetDeltaCounter(const TaskStatus_t *ts);


/*
static void cmd_A5(void)
{
    if (ads8325TaskHandle != NULL) {
        xTaskNotifyGive(ads8325TaskHandle);
    }
}
*/

// ===== CPU 런타임 카운터 기준값 리셋 (TC 명령) =====
static void TaskCpu_ResetAllBaselines(void)
{
    TaskStatus_t *statusArr = gTaskStatus;
    UBaseType_t n = uxTaskGetNumberOfTasks();
    if (n > TASKSTAT_MAX) n = TASKSTAT_MAX;

    configRUN_TIME_COUNTER_TYPE totalRunTime = 0;
    n = uxTaskGetSystemState(statusArr, n, &totalRunTime);

    gBaseTotalRunTime = totalRunTime;
    gBaseValid = 1;

    for (UBaseType_t i = 0; i < n; i++) {
        gTaskCpuBaseline[i].handle      = statusArr[i].xHandle;
        gTaskCpuBaseline[i].baseCounter = statusArr[i].ulRunTimeCounter;
    }
    for (UBaseType_t i = n; i < TASKSTAT_MAX; i++) {
        gTaskCpuBaseline[i].handle      = NULL;
        gTaskCpuBaseline[i].baseCounter = 0;
    }
}


// 기준값을 뺀 “delta” 카운터 얻기
static uint32_t TaskCpu_GetDeltaCounter(const TaskStatus_t *ts)
{
    if (!ts) return 0;

    for (UBaseType_t i = 0; i < TASKSTAT_MAX; i++) {
        if (gTaskCpuBaseline[i].handle == ts->xHandle) {
            uint32_t base = gTaskCpuBaseline[i].baseCounter;
            uint32_t now  = ts->ulRunTimeCounter;

            // wrap-aware delta
            return (uint32_t)(now - base);
            // unsigned subtraction은 wrap 포함해서 자동으로 올바른 delta가 나옵니다.
        }
    }

    // 기준값 없는 Task는 0으로 처리(“누적값 그대로”는 왜곡을 만들기 쉬움)
    return 0;
}


// ====== 번호로 TaskHandle 찾기: Task 이름이 "1_...", "2_..." 형식이라고 가정
static TaskHandle_t find_task_by_index(int idx)
{
    TaskStatus_t *statusArr = gTaskStatus;
    UBaseType_t n = uxTaskGetNumberOfTasks();
    if (n > TASKSTAT_MAX) n = TASKSTAT_MAX;

    n = uxTaskGetSystemState(statusArr, n, NULL);

    char prefix[8];
    snprintf(prefix, sizeof(prefix), "%d", idx);
    size_t plen = strlen(prefix);

    for (UBaseType_t i = 0; i < n; i++) {
        const char *name = statusArr[i].pcTaskName;
        if (!name) continue;

        // 이름이 "idx_..." 또는 "idx"로 시작하면 매칭
        if (strncmp(name, prefix, plen) == 0 &&
            (name[plen] == '_' || name[plen] == '\0')) {
            return statusArr[i].xHandle;
        }
    }
    return NULL;
}

// ===== 특정 Task는 보호해서 Kx/KA로 Suspend 못 하게 함
static bool is_task_protected_name(const char *name)
{
    if (!name) return true;

    // FreeRTOS 기본 IDLE Task
    if (strncmp(name, "IDLE", 4) == 0) return true;

    // Timer Service Task
    if (strncmp(name, "Tmr Svc", 7) == 0) return true;

    // UART 콘솔 Task (이름에 UARTCmd 들어가면 보호)
    if (strstr(name, "UARTCmd") != NULL) return true;

    return false;
}


// ==== TaskHandle로 Task 이름 찾기 (gTaskStatus 버퍼 사용)
static const char *get_task_name_by_handle(TaskHandle_t h)
{
    if (h == NULL) return NULL;

    TaskStatus_t *statusArr = gTaskStatus;
    UBaseType_t n = uxTaskGetNumberOfTasks();
    if (n > TASKSTAT_MAX) n = TASKSTAT_MAX;

    n = uxTaskGetSystemState(statusArr, n, NULL);

    for (UBaseType_t i = 0; i < n; i++) {
        if (statusArr[i].xHandle == h) {
            return statusArr[i].pcTaskName;
        }
    }
    return NULL;
}


// ================== 화면 클리어 + 배너 ==================
static void Terminal_ClearScreenAndBanner(void)
{
    // 1) 줄을 여러 개 보내서 기존 내용 위로 밀어버리기
    for (int i = 0; i < 40; i++) {
        const char *nl = "\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)nl, 2, HAL_MAX_DELAY);
    }

    // 2) 배너 출력
    const char *banner =
        "RF474 FreeRTOS Console \r\n";

    HAL_UART_Transmit(&huart1, (uint8_t*)banner, strlen(banner), HAL_MAX_DELAY);

    // 3) 프롬프트 출력
    const char *prompt = "> ";
    HAL_UART_Transmit(&huart1, (uint8_t*)prompt, strlen(prompt), HAL_MAX_DELAY);
}

// ================== Help 출력 처리 ==================

static void print_help(void)
{
    uart_write_str(
        "\r\n====== COMMAND HELP =====\r\n"
        "h, help  : Show this help\r\n"
        "\r\n"
        "Kx       : Suspend task whose name starts with 'x_'\r\n"
        "Rx       : Resume task whose name starts with 'x_'\r\n"
        "KA       : Suspend ALL numbered tasks (except protected)\r\n"
        "RA       : Resume  ALL numbered tasks (except protected)\r\n"
        "Xx       : Soft-restart task x (Suspend+Resume, Except protected Task)\r\n"
        "XA       : Soft-restart ALL numbered tasks (Except protected Task)\r\n"
        "\r\n"
        "TC       : Reset CPU runtime counters baseline (for 't')\r\n"
        "s        : Show PID + target status\r\n"
        "t        : Show FreeRTOS task stats (St=R/S)\r\n"
        "\r\n"
        "Pxxx     : Set Kp (P120 -> 0.120)\r\n"
        "Ixxx     : Set Ki (I050 -> 0.050)\r\n"
        "Dxxx     : Set Kd (D010 -> 0.010)\r\n"
        "Gxx      : Set target position (G35 -> 35000 ADC)\r\n"
        "R, RUN   : Move to saved G target\r\n"
        "\r\n"
        "v0       : V mode STOP\r\n"
        "v1       : V mode 1 Hz (25k <-> 45k)\r\n"
        "v2       : V mode 2 Hz (25k <-> 45k)\r\n"
        "v5       : V mode 5 Hz (25k <-> 45k)\r\n"
        "v10      : V mode 10 Hz (25k <-> 45k)\r\n"
        "\r\n"
        "========================\r\n"
    );
}



// ================== 사용자 명령 처리 ==================
// ================== 사용자 명령 처리 ==================
static void handle_user_command(const char *line)
{
    if (!line) return;

    // 앞쪽 공백/탭 무시 ( " ka" 같은 것도 인식되도록 )
    while (*line == ' ' || *line == '\t') {
        line++;
    }
    if (line[0] == 0) return;

    char cmd = line[0];

    /* ====== KA / RA : 전체 Task 제어 ====== */

    // ----- KA : 숫자로 시작하는 Task 전체 Suspend (보호 Task 제외) -----
    if ((cmd == 'K' || cmd == 'k') &&
        (line[1] == 'A' || line[1] == 'a') &&
        (line[2] == 0))
    {
        TaskStatus_t *statusArr = gTaskStatus;
        UBaseType_t n = uxTaskGetNumberOfTasks();
        if (n > TASKSTAT_MAX) n = TASKSTAT_MAX;

        n = uxTaskGetSystemState(statusArr, n, NULL);

        int suspended = 0;
        int skipped_protected = 0;

        for (UBaseType_t i = 0; i < n; i++) {
            const char *name = statusArr[i].pcTaskName;
            if (!name) continue;

            // 이름이 숫자로 시작하는 Task만 대상 ("1_LEDTask" 등)
            if (!isdigit((unsigned char)name[0])) {
                continue;
            }

            if (is_task_protected_name(name)) {
                skipped_protected++;
                continue;
            }

            if (statusArr[i].eCurrentState != eSuspended) {
                vTaskSuspend(statusArr[i].xHandle);
                suspended++;
            }
        }

        char buf[96];
        snprintf(buf, sizeof(buf),
                 "\r\nKA: suspended=%d, protected=%d\r\n",
                 suspended, skipped_protected);
        uart_write_str(buf);
        return;
    }

    // ----- RA : 숫자로 시작하는 Task 전체 Resume (보호 Task 제외) -----
    if ((cmd == 'R' || cmd == 'r') &&
        (line[1] == 'A' || line[1] == 'a') &&
        (line[2] == 0))
    {
        TaskStatus_t *statusArr = gTaskStatus;
        UBaseType_t n = uxTaskGetNumberOfTasks();
        if (n > TASKSTAT_MAX) n = TASKSTAT_MAX;

        n = uxTaskGetSystemState(statusArr, n, NULL);

        int resumed = 0;
        int skipped_protected = 0;

        for (UBaseType_t i = 0; i < n; i++) {
            const char *name = statusArr[i].pcTaskName;
            if (!name) continue;

            // 이름이 숫자로 시작하는 Task만 대상
            if (!isdigit((unsigned char)name[0])) {
                continue;
            }

            if (is_task_protected_name(name)) {
                skipped_protected++;
                continue;
            }

            if (statusArr[i].eCurrentState == eSuspended) {
                vTaskResume(statusArr[i].xHandle);
                resumed++;
            }
        }

        char buf[96];
        snprintf(buf, sizeof(buf),
                 "\r\nRA: resumed=%d, protected=%d\r\n",
                 resumed, skipped_protected);
        uart_write_str(buf);
        return;
    }

    /* ====== Kx / Rx : 개별 Task 제어 ====== */

    // ----- Kx : 특정 번호 Task Suspend -----
    if (cmd == 'K' || cmd == 'k')
    {
        const char *numstr = &line[1];
        if (*numstr == 0 || !isdigit((unsigned char)*numstr)) {
            uart_write_str("\r\nERR: Kx -> x는 숫자여야 합니다\r\n");
            return;
        }

        int idx = atoi(numstr);
        if (idx <= 0) {
            uart_write_str("\r\nERR: Task index must be > 0\r\n");
            return;
        }

        TaskHandle_t h = find_task_by_index(idx);
        if (h == NULL) {
            uart_write_str("\r\nERR: 해당 번호의 Task를 찾을 수 없습니다\r\n");
            return;
        }

        const char *name = get_task_name_by_handle(h);
        if (is_task_protected_name(name)) {
            uart_write_str("\r\nERR: 보호 Task는 Suspend 할 수 없습니다\r\n");
            return;
        }

        vTaskSuspend(h);

        char buf[64];
        snprintf(buf, sizeof(buf),
                 "\r\nK%d: Task suspended\r\n", idx);
        uart_write_str(buf);
        return;
    }

    // ----- Xx : 특정 번호 Task Soft-Restart (Suspend+Resume) -----
    if (cmd == 'X' || cmd == 'x')
    {
        const char *numstr = &line[1];
        if (*numstr == 0 || !isdigit((unsigned char)*numstr)) {
            uart_write_str("\r\nERR: Xx -> x는 숫자여야 합니다\r\n");
            return;
        }

        int idx = atoi(numstr);
        if (idx <= 0) {
            uart_write_str("\r\nERR: Task index must be > 0\r\n");
            return;
        }

        TaskHandle_t h = find_task_by_index(idx);
        if (h == NULL) {
            uart_write_str("\r\nERR: 해당 번호의 Task를 찾을 수 없습니다\r\n");
            return;
        }

        const char *name = get_task_name_by_handle(h);
        if (is_task_protected_name(name)) {
            uart_write_str("\r\nERR: 보호 Task는 X로 재시작할 수 없습니다\r\n");
            return;
        }

        eTaskState st = eTaskGetState(h);
        const char *msg_mode = NULL;

        if (st == eSuspended) {
            vTaskResume(h);
            msg_mode = "resumed (was suspended)";
        } else {
            vTaskSuspend(h);
            vTaskResume(h);
            msg_mode = "soft-restarted (Suspend+Resume)";
        }

        char buf[96];
        snprintf(buf, sizeof(buf),
                 "\r\nX%d: Task %s\r\n", idx, msg_mode);
        uart_write_str(buf);
        return;
    }


    // ----- Rx : 특정 번호 Task Resume -----
    if ((cmd == 'R' || cmd == 'r') &&
        isdigit((unsigned char)line[1]))
    {
        const char *numstr = &line[1];
        int idx = atoi(numstr);
        if (idx <= 0) {
            uart_write_str("\r\nERR: Task index must be > 0\r\n");
            return;
        }

        TaskHandle_t h = find_task_by_index(idx);
        if (h == NULL) {
            uart_write_str("\r\nERR: 해당 번호의 Task를 찾을 수 없습니다\r\n");
            return;
        }

        // 보호 Task라도 Resume은 허용
        vTaskResume(h);

        char buf[64];
        snprintf(buf, sizeof(buf),
                 "\r\nR%d: Task resumed\r\n", idx);
        uart_write_str(buf);
        return;
    }

    // ----- XA : 숫자로 시작하는 Task 전체 Soft-Restart (Suspend+Resume) -----
        if ((cmd == 'X' || cmd == 'x') &&
            (line[1] == 'A' || line[1] == 'a') &&
            (line[2] == 0))
        {
            TaskStatus_t *statusArr = gTaskStatus;
            UBaseType_t n = uxTaskGetNumberOfTasks();
            if (n > TASKSTAT_MAX) n = TASKSTAT_MAX;

            n = uxTaskGetSystemState(statusArr, n, NULL);

            int restarted = 0;
            int resumed_only = 0;
            int skipped_protected = 0;

            for (UBaseType_t i = 0; i < n; i++) {
                const char *name = statusArr[i].pcTaskName;
                if (!name) continue;

                // 숫자로 시작하는 Task만 대상
                if (!isdigit((unsigned char)name[0])) {
                    continue;
                }

                if (is_task_protected_name(name)) {
                    skipped_protected++;
                    continue;
                }

                TaskHandle_t h = statusArr[i].xHandle;
                eTaskState st = eTaskGetState(h);

                if (st == eSuspended) {
                    vTaskResume(h);
                    resumed_only++;
                } else {
                    vTaskSuspend(h);
                    vTaskResume(h);
                    restarted++;
                }
            }

            char buf[96];
            snprintf(buf, sizeof(buf),
                     "\r\nXA: restarted=%d, resumed_only=%d, protected=%d\r\n",
                     restarted, resumed_only, skipped_protected);
            uart_write_str(buf);
            return;
        }

        /* ====== TC : CPU 런타임 카운터 기준 리셋 ====== */
        if ((cmd == 'T' || cmd == 't') &&
            (line[1] == 'C' || line[1] == 'c') &&
            (line[2] == 0))
        {
            TaskCpu_ResetAllBaselines();
            uart_write_str("\r\nTC: CPU counters baseline reset for 't' stats\r\n");
            return;
        }



        //=== 기존 명령 =====

    // --------- HELP: h / help ----------
    if ((cmd == 'h' || cmd == 'H') ||
        (strcmp(line, "help") == 0) ||
        (strcmp(line, "HELP") == 0))
    {
        print_help();
        return;
    }

    // --------- PID 설정: P / I / D ----------
    if (cmd == 'P' || cmd == 'p' ||
        cmd == 'I' || cmd == 'i' ||
        cmd == 'D' || cmd == 'd')
    {
        const char *numstr = &line[1];
        if (*numstr == 0) {
            uart_write_str("\r\nERR: no number\r\n");
            return;
        }

        long val = strtol(numstr, NULL, 10);   // 10진수

        float kp, ki, kd;
        VCA_PID_GetGains(&kp, &ki, &kd);

        if (cmd == 'P' || cmd == 'p') {
            kp = (float)val / 1000.0f;
        } else if (cmd == 'I' || cmd == 'i') {
            ki = (float)val / 1000.0f;
        } else { // D
            kd = (float)val / 1000.0f;
        }

        VCA_PID_SetGains(kp, ki, kd);

        char buf[96];
        long kp_milli = (long)(kp * 1000.0f);
        long ki_milli = (long)(ki * 1000.0f);
        long kd_milli = (long)(kd * 1000.0f);

        snprintf(buf, sizeof(buf),
                 "\r\nPID updated: Kp=%ld/1000, Ki=%ld/1000, Kd=%ld/1000\r\n",
                 kp_milli, ki_milli, kd_milli);

        uart_write_str(buf);
        return;
    }

    // --------- G 값만 세팅 (이동은 하지 않음) ----------
    if (cmd == 'G' || cmd == 'g')
    {
        const char *numstr = &line[1];
        if (*numstr == 0) {
            uart_write_str("\r\nERR: no number\r\n");
            return;
        }

        long gval = strtol(numstr, NULL, 10);  // 예: 35
        if (gval < 0)  gval = 0;
        if (gval > 65) gval = 65;              // ADC 0~65535 가드

        uint16_t adc = (uint16_t)(gval * 1000L);

        // pending 값 갱신
        s_vca_pending_target_adc = adc;

        // 실제 PID 타겟에도 바로 적용
        VCA_SetTargetADC(adc);

        char buf[96];
        snprintf(buf, sizeof(buf),
                 "\r\nG saved: G=%ld -> pending_adc=%u\r\n",
                 gval, (unsigned)adc);
        uart_write_str(buf);
        return;
    }

    // --------- RUN: 저장된 G 값으로 실제 이동 시작 ----------
    // "RUN", "Run", "run", 또는 간단히 "R"/"r" 도 허용
    if ((cmd == 'R' || cmd == 'r') &&
        (line[1] == 0 || (line[1]=='U' || line[1]=='u')))
    {
        uint16_t adc = s_vca_pending_target_adc;

        VCA_SetTargetADC(adc);   // 여기서부터 PID 타겟으로 사용 시작

        char buf[96];
        snprintf(buf, sizeof(buf),
                 "\r\nRUN: target_adc=%u\r\n",
                 (unsigned)adc);
        uart_write_str(buf);
        return;
    }

    // --------- V MODE: 25k <-> 45k f Hz 왕복 ----------
    if (cmd == 'V' || cmd == 'v')
    {
        const char *numstr = &line[1];
        int f = atoi(numstr);   // v1, v2, v5, v10 → 주파수

        // v0 → STOP
        if (f <= 0)
        {
            s_vca_vmode_enable = false;
            uart_write_str("\r\nV MODE STOP\r\n");
            return;
        }

        // 주파수 상한 = 10 Hz
        if (f > 10) f = 10;

        // f Hz → 반주기(ms) = 500 / f
        s_vca_v_half_period_ms = 500 / f;

        s_vca_vmode_enable  = true;
        s_vca_v_last_tick   = osKernelGetTickCount();

        // 항상 LOW 위치에서 시작
        VCA_SetTargetADC(s_vca_v_low);

        char msg[64];
        snprintf(msg, sizeof(msg),
                 "\r\nV MODE START: 25k <-> 45k @%dHz\r\n", f);
        uart_write_str(msg);
        return;
    }

    // --------- 상태 출력: s / S ----------
    if (cmd == 's' || cmd == 'S')
    {
        float kp, ki, kd;
        VCA_PID_GetGains(&kp, &ki, &kd);
        uint16_t cur_tgt = VCA_GetTargetADC();

        char buf[192];
        long kp_milli = (long)(kp * 1000.0f);
        long ki_milli = (long)(ki * 1000.0f);
        long kd_milli = (long)(kd * 1000.0f);

        snprintf(buf, sizeof(buf),
                 "\r\n[PID STATUS]\r\n"
                 "Kp          = %ld/1000\r\n"
                 "Ki          = %ld/1000\r\n"
                 "Kd          = %ld/1000\r\n"
                 "G(pending)  = %u (ADC)\r\n"
                 "Target(NOW) = %u (ADC)\r\n",
                 kp_milli, ki_milli, kd_milli,
                 (unsigned)s_vca_pending_target_adc,
                 (unsigned)cur_tgt);
        uart_write_str(buf);

        return;
    }

    uart_write_str("\r\nUnknown cmd\r\n");
}


static int task_sort_key(const TaskStatus_t *ts)
{
    const char *name = ts->pcTaskName;
    if (strncmp(name, "IDLE", 4) == 0)     return -1;      // 최상단
    if (!name || name[0] == 0) return 1000000;
    if (strncmp(name, "Tmr Svc", 7) == 0)  return 1000001; // 최하단

    // 번호로 시작하면 그 번호를 key로
    if (name[0] >= '0' && name[0] <= '9') {
        int v = 0;
        int i = 0;
        while (name[i] >= '0' && name[i] <= '9') {
            v = v * 10 + (name[i] - '0');
            i++;
        }
        // "12_" 또는 "12" 형태만 번호로 인정
        if (name[i] == '_' || name[i] == '\0') return v;
    }

    // 나머지는 뒤로 (IDLE/Tmr Svc 포함)
    // 필요하면 여기서 IDLE=999998, Tmr=999999 같이 더 세밀하게 가능
    return 999999;
}

static void sort_task_status(TaskStatus_t *arr, UBaseType_t n)
{
    // 간단한 선택정렬(작은 n에서 충분)
    for (UBaseType_t i = 0; i + 1 < n; i++) {
        UBaseType_t min = i;
        int kmin = task_sort_key(&arr[min]);
        for (UBaseType_t j = i + 1; j < n; j++) {
            int kj = task_sort_key(&arr[j]);
            if (kj < kmin) {
                min = j;
                kmin = kj;
            }
        }
        if (min != i) {
            TaskStatus_t tmp = arr[i];
            arr[i] = arr[min];
            arr[min] = tmp;
        }
    }
}


// ================== 통계용 헬퍼 ==================
static void print_task_stats(UART_HandleTypeDef *huart)
{
    (void)huart;

    TaskStatus_t *statusArr = gTaskStatus;
    UBaseType_t n = uxTaskGetNumberOfTasks();
    if (n > TASKSTAT_MAX) n = TASKSTAT_MAX;

   // uint32_t totalRunTime = 0;
    configRUN_TIME_COUNTER_TYPE totalRunTime = 0;

    n = uxTaskGetSystemState(statusArr, n, &totalRunTime);
    sort_task_status(statusArr, n);

    if (!gBaseValid) {
        TaskCpu_ResetAllBaselines();
        uart_write_str("\r\n[TASK STATS] primed. Press t again.\r\n");
        return;
    }

    uint32_t dTotal = (uint32_t)(totalRunTime - gBaseTotalRunTime);
    if (dTotal == 0) dTotal = 1;

    uart_write_str("\r\nName              Prio St   CPU%   HWM(bytes)   StackFree\r\n");
    uart_write_str(  "--------------------------------------------------------------\r\n");

    for (UBaseType_t i = 0; i < n; i++) {
        uint32_t dTask = TaskCpu_GetDeltaCounter(&statusArr[i]);
        uint32_t cpu10 = (uint32_t)((1000ULL * (uint64_t)dTask) / (uint64_t)dTotal);

        char st = (statusArr[i].eCurrentState == eSuspended) ? 'S' : 'R';

        char line[96];
        snprintf(line, sizeof(line),
                 "%-16s %4lu  %c %6lu.%1lu %12u %11u\r\n",
                 statusArr[i].pcTaskName,
                 (unsigned long)statusArr[i].uxBasePriority,
                 st,
                 (unsigned long)(cpu10 / 10), (unsigned long)(cpu10 % 10),
                 (unsigned)(statusArr[i].usStackHighWaterMark * sizeof(StackType_t)),
                 (unsigned)statusArr[i].usStackHighWaterMark);

        uart_write_str(line);
    }
}


static void print_heap_stats(void)
{
    char buf[96];

#if (configUSE_HEAP_SCHEME == 4 || configUSE_HEAP_SCHEME == 5)
    HeapStats_t hs;
    vPortGetHeapStats(&hs);
    snprintf(buf, sizeof(buf),
             "Heap Free(bytes):      %lu\r\n"
             "Heap MinEverFree(bytes): %lu\r\n",
             (unsigned long)hs.xAvailableHeapSpaceInBytes,
             (unsigned long)hs.xMinimumEverFreeBytesRemaining);
#else
    extern size_t xPortGetFreeHeapSize(void);
    extern size_t xPortGetMinimumEverFreeHeapSize(void);
    snprintf(buf, sizeof(buf),
             "Heap Free(bytes):      %lu\r\n"
             "Heap MinEverFree(bytes): %lu\r\n",
             (unsigned long)xPortGetFreeHeapSize(),
             (unsigned long)xPortGetMinimumEverFreeHeapSize());
#endif
    uart_write_str(buf);
}

// ================== RX 큐 상태 조회 (외부용) ==================
size_t UART_Cmd_GetRxQueueFree(void)
{
    if (s_rxq == NULL) return 0;
    return (size_t)uxQueueSpacesAvailable(s_rxq);
}

size_t UART_Cmd_GetRxQueueLength(void)
{
    return (size_t)RXQ_LEN;
}

// ================== 로컬 유틸 ==================
static void uart_write_str(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

static void uart_write_len(const uint8_t *buf, size_t n)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);
}

static void uart_send_data(const uint8_t *buf, uint16_t len)
{
    uart_write_len(buf, (size_t)len);
}

static void uart_echo_byte(uint8_t ch)
{
    if (ch == '\r' || ch == '\n') {
        const char crlf[2] = {'\r','\n'};
        uart_send_data((const uint8_t*)crlf, 2);
    } else if (ch == 0x08 || ch == 0x7F) { /* Backspace/DEL */
        const char bs_seq[3] = {'\b',' ','\b'};
        uart_send_data((const uint8_t*)bs_seq, 3);
    } else {
        uart_send_data(&ch, 1);
    }
}

// ================== RX 콜백 ==================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        BaseType_t xHptw = pdFALSE;
        xQueueSendFromISR(s_rxq, &s_rx_byte, &xHptw);
        HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1); // 재예약 필수
        portYIELD_FROM_ISR(xHptw);
    }
}

// ================== 초기화 (한 번만) ==================
void UART_Cmd_Init(void)
{
    if (sUartCmdTaskHandle != NULL) return; // 중복 생성 방지

    s_rxq = xQueueCreate(RXQ_LEN, sizeof(uint8_t));
    configASSERT(s_rxq != NULL);

    // UART1 RX 인터럽트 시작
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);

    // Command Task 생성 (FreeRTOS native)
    // NOTE: 't' 명령/printf/snprintf 등으로 스택을 꽤 사용하므로 여유 있게.
    // xTaskCreate의 stackDepth는 'words' 단위입니다( Cortex-M: 1word=4bytes ).
    xTaskCreate(UART_CmdTask, "0_UARTCmd", 128*3, NULL,
                osPriorityNormal, &sUartCmdTaskHandle);
}

// ================== Command Task ==================
void UART_CmdTask(void *arg)
{
    (void)arg;
    char   linebuf[CMD_LINE_MAX];
    size_t linelen = 0;
    bool   last_was_nl = false;

    // RTOS/USART 초기화가 끝난 후 콘솔 화면 정리 + 배너 + 프롬프트
    osDelay(200);  // 200ms 정도 여유
    Terminal_ClearScreenAndBanner();

    // ===== 리셋 직후: Task Stats를 자동 1회 출력 =====
    TaskCpu_ResetAllBaselines();
    vTaskDelay(pdMS_TO_TICKS(1000));
    uart_write_str("\r\n[TASK STATS]\r\n");
    print_task_stats(&huart1);
    print_heap_stats();
    uart_write_str("> ");

    for (;;)
    {
        /* ===== V MODE 2Hz OSCILLATION ===== */
    	if (s_vca_vmode_enable)
    	{
    	    uint32_t now = osKernelGetTickCount();

    	    if (now - s_vca_v_last_tick >= s_vca_v_half_period_ms)
    	    {
    	        s_vca_v_last_tick = now;

    	        static bool v_dir = false;
    	        v_dir = !v_dir;

    	        if (v_dir)
    	            VCA_SetTargetADC(s_vca_v_high);  // 45k
    	        else
    	            VCA_SetTargetADC(s_vca_v_low);   // 25k
    	    }
    	}



        // === UART 입력 (10ms 타임아웃) ===
        uint8_t ch;
        if (xQueueReceive(s_rxq, &ch, pdMS_TO_TICKS(10)) != pdTRUE) {
            // 입력이 없으면 다시 V MODE만 체크
            continue;
        }

        // 개행 처리: CR, LF 모두 라인 종료로 인정
        if (ch == '\r' || ch == '\n') {
            // CR 다음 바로 LF(또는 반대)면 두 번째는 무시
            if (last_was_nl) {
                last_was_nl = false;
                continue;
            }
            last_was_nl = true;

            uart_echo_byte(ch);  // 화면엔 항상 \r\n

            linebuf[linelen] = 0;

            // ---- 명령 실행 ----
            if (linelen == 1 &&
                (linebuf[0] == 't' || linebuf[0] == 'T'))
            {
                // 1초 고정 윈도우로 측정 (t 출력 때문에 UARTCmd% 튀는 현상 완화)
                TaskCpu_ResetAllBaselines();
                vTaskDelay(pdMS_TO_TICKS(1000));

                uart_write_str("\r\n[TASK STATS]\r\n");
                print_task_stats(&huart1);
                print_heap_stats();
            }
            else if (linelen > 0)
            {
                // ✅ 이 줄이 있어야 h/help, P/I/D/G/RUN/v/s, TC 등이 동작합니다
                handle_user_command(linebuf);
            }

            // 다음 줄 준비 + 프롬프트
            linelen = 0;
            uart_write_str("> ");
            continue;


        } else {
            last_was_nl = false;     // 일반 문자면 NL 연속 상태 해제
        }

        // 편집키: Backspace/DEL
        if (ch == 0x08 || ch == 0x7F) {
            if (linelen > 0) {
                linelen--;
                uart_echo_byte(ch);
            }
            continue;
        }

        // 인쇄 가능 문자만 수집
        if (ch >= 0x20 && ch <= 0x7E) {
            if (linelen < CMD_LINE_MAX - 1) {
                linebuf[linelen++] = (char)ch;
                uart_echo_byte(ch);
            } else {
                // 넘치면 벨 등으로 알릴 수도 있음
            }
        }
    }
}
