/*===========================================================================
 * [app_main_runtime.h] main loop 运行时辅助接口
 *
 * 封装 PIT tick 等待、stdout 输出、心跳日志、UART4 原始字节探针等与主循环
 * 调度有关的轻量功能。中断侧只做计数/缓存，复杂业务留给主循环处理。
 *===========================================================================*/

#ifndef APP_MAIN_RUNTIME_H_
#define APP_MAIN_RUNTIME_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 函数: App_MainRuntime_SetupStdout
 * 功能: 关闭 stdout 缓冲，使 printf 日志能立即从串口/调试通道吐出。
 */
void App_MainRuntime_SetupStdout(void);

/*
 * 函数: App_MainRuntime_PrintLoopEnteredOnce
 * 功能: 主循环首次进入时打印一次诊断信息，确认 PIT 与 main loop 已跑起来。
 */
void App_MainRuntime_PrintLoopEnteredOnce(void);

/*
 * 函数: App_MainRuntime_PrintTickHeartbeat
 * 功能: 周期性打印 tick 心跳，用于观察主循环是否卡死。
 */
void App_MainRuntime_PrintTickHeartbeat(void);

/*
 * 函数: App_MainRuntime_WaitForTick
 * 功能: 等待 PIT tick 到来并返回积压 tick 数，主循环用它驱动固定周期任务。
 * 返回: 本轮需要补处理的 tick 数；>1 说明主循环曾经落后。
 */
uint32 App_MainRuntime_WaitForTick(void);

/*
 * 函数: App_MainRuntime_DrainUart4Tap5ms
 * 功能: STATIC_MAP_DRIVE 等待地图阶段，用于统计 UART4 原始字节流是否进入。
 */
void App_MainRuntime_DrainUart4Tap5ms(void);

/*
 * 函数: App_MainRuntime_DisableUart4Tap
 * 功能: 地图锁定后关闭 UART4 原始流探针，避免无意义日志持续刷屏。
 */
void App_MainRuntime_DisableUart4Tap(void);

/*
 * 函数: main_loop_on_pit_tick
 * 功能: PIT 中断回调入口：只累加 tick，不在中断中执行复杂业务。
 */
void main_loop_on_pit_tick(void);

/*
 * 函数: main_uart4_tap_byte
 * 功能: UART4 接收中断调试探针入口，缓存原始字节供主循环统计。
 */
void main_uart4_tap_byte(uint8 b);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_RUNTIME_H_ */
