/*===========================================================================
 * [app_main_runtime.c] main loop 运行时辅助实现
 *
 * 该模块是 main.c 与硬件中断之间的缓冲层：
 *   - PIT 中断累加 tick，主循环按固定周期消费
 *   - stdout 设置为实时输出
 *   - STATIC_MAP_DRIVE 等待地图阶段统计 UART4 原始输入
 *===========================================================================*/

#include "app_main_runtime.h"
#include "app_main_config.h"
#include "zf_common_headfile.h"
#include <stdio.h>

/* PIT tick 只在中断里累加，主循环一次性取走，避免中断内执行复杂逻辑。 */
static volatile uint32 s_main_tick_pending = 0U;
/* 记录主循环来不及消费的 tick 数，便于判断任务是否超时。 */
static volatile uint32 s_main_tick_overrun = 0U;
static volatile uint32 s_main_tick_total   = 0U;

#if (MAIN_RUN_MODE == MAIN_RUN_MODE_STATIC_MAP_DRIVE)
/* UART4 tap 是 STATIC_MAP_DRIVE 等待地图阶段的轻量调试环形缓冲。
 * 它只统计是否收到原始字节和是否丢字节，不参与正式协议解析。
 */
#define MAIN_UART4_TAP_BUF_LEN   (1024U)
static volatile uint8  s_u4tap_buf[MAIN_UART4_TAP_BUF_LEN];
static volatile uint8  s_u4tap_enable = 1U;
static volatile uint16 s_u4tap_head = 0U;
static volatile uint16 s_u4tap_tail = 0U;
static volatile uint32 s_u4tap_drop = 0U;
#endif

/*
 * 函数: App_MainRuntime_SetupStdout
 * 功能: 配置 stdout 为无缓冲输出。
 * 参数: 无。
 * 返回: 无。
 * 调用时机: main 初始化早期调用一次。
 * 说明: 嵌入式串口日志通常希望 printf 立即输出，否则缓冲区可能迟迟不刷出。
 */
void App_MainRuntime_SetupStdout(void)
{
    /* 嵌入式串口日志希望实时输出；单字节 buffer + _IONBF 基本等价于无缓冲。 */
    static char s_stdout_buf;
    setvbuf(stdout, &s_stdout_buf, _IONBF, sizeof(s_stdout_buf));
}

/*
 * 函数: main_loop_on_pit_tick
 * 功能: PIT 定时中断回调入口，通知主循环有新的调度 tick。
 * 参数: 无。
 * 返回: 无。
 * ISR 约束: 只累加 volatile 计数，不调用 printf、不做规划、不访问耗时外设。
 */
void main_loop_on_pit_tick(void)
{
    /* 中断入口只做计数，实际任务统一放到 main loop，降低 ISR 抖动。 */
    s_main_tick_pending++;
}

/*
 * 函数: main_uart4_tap_byte
 * 功能: UART4 接收中断中的原始字节探针。
 * 参数: b - 刚收到的 1 字节串口数据。
 * 返回: 无。
 * 用途: STATIC_MAP_DRIVE 等待地图时统计 UART4 是否有数据进入，辅助排查接线/波特率问题。
 * ISR 约束: 只写环形缓冲和计数器；若缓冲满则丢弃并累计 drop。
 */
void main_uart4_tap_byte(uint8 b)
{
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_STATIC_MAP_DRIVE)
    uint16 next = (uint16)(s_u4tap_head + 1U);
    if (s_u4tap_enable == 0U) { return; }
    if (next >= MAIN_UART4_TAP_BUF_LEN) next = 0U;
    if (next == s_u4tap_tail)
    {
        /* 环形缓冲满时丢弃本字节，仅累计丢包数用于日志诊断。 */
        s_u4tap_drop++;
        return;
    }
    s_u4tap_buf[s_u4tap_head] = b;
    s_u4tap_head = next;
#else
    (void)b;
#endif
}

/*
 * 函数: App_MainRuntime_DrainUart4Tap5ms
 * 功能: 在主循环中消费 UART4 原始字节探针统计。
 * 参数: 无。
 * 返回: 无。
 * 调用时机: STATIC_MAP_DRIVE 的 WAIT_MAP 阶段，每 5ms 调用。
 * 说明: 只打印本周期收到的字节数和累计丢弃数，不打印内容，避免日志阻塞主循环。
 */
void App_MainRuntime_DrainUart4Tap5ms(void)
{
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_STATIC_MAP_DRIVE)
    uint16 head, tail;
    static uint32 s_last_drop = 0U;

    /* volatile head/tail 先快照到局部变量，避免打印长度时被 ISR 改动。 */
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
    /* 这里只关心“这 5ms 是否有原始字节”，不逐字节打印，避免刷爆串口。 */
    s_u4tap_tail = head;

    if (s_u4tap_drop != s_last_drop)
    {
        s_last_drop = s_u4tap_drop;
        printf("SMD_RAW_DROP=%lu\n", (unsigned long)s_last_drop);
    }
#endif
}

/*
 * 函数: App_MainRuntime_DisableUart4Tap
 * 功能: 关闭 UART4 原始字节探针。
 * 参数: 无。
 * 返回: 无。
 * 调用时机: 地图已经锁定并进入执行阶段后调用。
 * 说明: 正式协议解析仍由 app_link 负责；关闭 tap 只是不再缓存原始调试字节。
 */
void App_MainRuntime_DisableUart4Tap(void)
{
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_STATIC_MAP_DRIVE)
    s_u4tap_enable = 0U;
#endif
}

/*
 * 函数: App_MainRuntime_WaitForTick
 * 功能: 等待 PIT tick，并原子取走当前积压 tick 数。
 * 参数: 无。
 * 返回: 自上次调用后累计的 tick 数；正常情况下为 1，>1 表示主循环发生超时积压。
 * 并发说明: s_main_tick_pending 由 PIT ISR 写入，本函数在关中断临界区中清零读取。
 */
uint32 App_MainRuntime_WaitForTick(void)
{
    uint32 ticks;
    while (s_main_tick_pending == 0U)
    {
        /* __WFI(); disabled while debugging SWD connection stability. */
    }
    __disable_irq();
    ticks = s_main_tick_pending;
    s_main_tick_pending = 0U;
    __enable_irq();
    if (ticks > 1U)
    {
        /* ticks>1 表示主循环某次执行超过了一个 PIT 周期。 */
        s_main_tick_overrun += (ticks - 1U);
    }
    s_main_tick_total += ticks;
    return ticks;
}

/*
 * 函数: App_MainRuntime_PrintLoopEnteredOnce
 * 功能: 主循环首次进入时打印一次 LOOP_OK。
 * 参数: 无。
 * 返回: 无。
 * 用途: 判断固件是否已经越过初始化阶段并进入 main loop。
 */
void App_MainRuntime_PrintLoopEnteredOnce(void)
{
    static uint8 s_loop_diag = 0U;
    if (!s_loop_diag)
    {
        s_loop_diag = 1U;
        uart_write_string(UART_1, "LOOP_OK\r\n");
    }
}

/*
 * 函数: App_MainRuntime_PrintTickHeartbeat
 * 功能: 周期性打印主循环心跳。
 * 参数: 无。
 * 返回: 无。
 * 调度: 每调用 200 次打印一次；若主循环为 5ms tick，则约 1s 一次。
 */
void App_MainRuntime_PrintTickHeartbeat(void)
{
    static uint32 s_tick_cnt = 0U;
    s_tick_cnt++;
    if ((s_tick_cnt % 200U) == 0U)
    {
        printf("T:%lu\n", (unsigned long)(s_tick_cnt / 200U));
    }
}
