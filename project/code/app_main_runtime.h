/*
 * app_main_runtime.h
 *
 * main.c 与中断共享的轻量运行时辅助接口。
 * 面向 ISR 的函数只修改 volatile 计数器或环形缓冲索引，耗时工作留在主循环。
 */

#ifndef APP_MAIN_RUNTIME_H_
#define APP_MAIN_RUNTIME_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

void App_MainRuntime_SetupStdout(void);
void App_MainRuntime_PrintLoopEnteredOnce(void);
void App_MainRuntime_PrintTickHeartbeat(void);

/* 等待至少一个 5ms PIT tick，并一次性消费已累计的 tick 数。 */
uint32 App_MainRuntime_WaitForTick(void);

/* 开启或关闭静态地图调试阶段使用的 UART4 原始字节探针。 */
void App_MainRuntime_EnableUart4Tap(void);
void App_MainRuntime_DisableUart4Tap(void);

/* 在主循环中排空 UART4 探针统计；禁止在中断里调用。 */
void App_MainRuntime_DrainUart4Tap5ms(void);

/* PIT 中断钩子：只递增 volatile tick 计数。 */
void main_loop_on_pit_tick(void);

/* UART4 中断钩子：探针开启时把 1 字节放入诊断环形缓冲。 */
void main_uart4_tap_byte(uint8 b);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_RUNTIME_H_ */
