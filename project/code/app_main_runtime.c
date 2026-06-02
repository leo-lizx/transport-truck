/*
 * app_main_runtime.c
 *
 * 主循环节拍与轻量中断桥接实现。
 * 共享变量使用 volatile，因为 PIT/UART 中断写入，主循环读取。
 * 中断中禁止 printf、Flash 写入、路径规划和阻塞等待。
 */

#include "app_main_runtime.h"
#include "zf_common_headfile.h"
#include <stdio.h>

#define MAIN_UART4_TAP_BUF_LEN   (1024U)

static volatile uint32 s_main_tick_pending = 0U;
static volatile uint32 s_main_tick_overrun = 0U;
static volatile uint32 s_main_tick_total   = 0U;

static volatile uint8  s_u4tap_buf[MAIN_UART4_TAP_BUF_LEN];
static volatile uint8  s_u4tap_enable = 0U;
static volatile uint16 s_u4tap_head = 0U;
static volatile uint16 s_u4tap_tail = 0U;
static volatile uint32 s_u4tap_drop = 0U;

void App_MainRuntime_SetupStdout(void)
{
    static char s_stdout_buf;
    setvbuf(stdout, &s_stdout_buf, _IONBF, sizeof(s_stdout_buf));
}

void main_loop_on_pit_tick(void)
{
    s_main_tick_pending++;
}

void main_uart4_tap_byte(uint8 b)
{
    uint16 next;

    if (s_u4tap_enable == 0U)
    {
        return;
    }

    next = (uint16)(s_u4tap_head + 1U);
    if (next >= MAIN_UART4_TAP_BUF_LEN)
    {
        next = 0U;
    }
    if (next == s_u4tap_tail)
    {
        s_u4tap_drop++;
        return;
    }

    s_u4tap_buf[s_u4tap_head] = b;
    s_u4tap_head = next;
}

void App_MainRuntime_EnableUart4Tap(void)
{
    __disable_irq();
    s_u4tap_head = 0U;
    s_u4tap_tail = 0U;
    s_u4tap_drop = 0U;
    s_u4tap_enable = 1U;
    __enable_irq();
}

void App_MainRuntime_DisableUart4Tap(void)
{
    s_u4tap_enable = 0U;
}

void App_MainRuntime_DrainUart4Tap5ms(void)
{
    uint16 head;
    uint16 tail;
    static uint32 s_last_drop = 0U;

    if (s_u4tap_enable == 0U)
    {
        return;
    }

    head = s_u4tap_head;
    tail = s_u4tap_tail;

    if (head == tail)
    {
        if (s_u4tap_drop != s_last_drop)
        {
            s_last_drop = s_u4tap_drop;
            printf("SMD_RAW_DROP=%lu\n", (unsigned long)s_last_drop);
        }
        return;
    }

    printf("SMD_RAW len=%u\n",
           (unsigned)((head >= tail) ? (uint16)(head - tail) :
                                       (uint16)(MAIN_UART4_TAP_BUF_LEN - tail + head)));
    s_u4tap_tail = head;

    if (s_u4tap_drop != s_last_drop)
    {
        s_last_drop = s_u4tap_drop;
        printf("SMD_RAW_DROP=%lu\n", (unsigned long)s_last_drop);
    }
}

uint32 App_MainRuntime_WaitForTick(void)
{
    uint32 ticks;

    while (s_main_tick_pending == 0U)
    {
        /* 调试 SWD 稳定性期间暂不进入 __WFI，避免影响在线调试连接。 */
    }

    __disable_irq();
    ticks = s_main_tick_pending;
    s_main_tick_pending = 0U;
    __enable_irq();

    if (ticks > 1U)
    {
        s_main_tick_overrun += (ticks - 1U);
    }
    s_main_tick_total += ticks;
    return ticks;
}

void App_MainRuntime_PrintLoopEnteredOnce(void)
{
    static uint8 s_loop_diag = 0U;

    if (s_loop_diag == 0U)
    {
        s_loop_diag = 1U;
        uart_write_string(UART_1, "LOOP_OK\r\n");
    }
}

void App_MainRuntime_PrintTickHeartbeat(void)
{
    static uint32 s_tick_cnt = 0U;

    s_tick_cnt++;
    if ((s_tick_cnt % 200U) == 0U)
    {
        printf("T:%lu\n", (unsigned long)(s_tick_cnt / 200U));
    }
}
