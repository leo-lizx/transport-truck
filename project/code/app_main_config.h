/*
 * app_main_config.h
 *
 * main.c 与运行模式模块共享的系统级常量。
 * 本文件只保留结构性常量和固定默认值；运行时选择与人工调参量放入
 * app_main_options_t 或 chassis_tune_params_t。
 */

#ifndef APP_MAIN_CONFIG_H_
#define APP_MAIN_CONFIG_H_

#include "chassis_config.h"
#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAIN_OPENART1_UART             (UART_4)
#define MAIN_OPENART1_UART_TX          (UART4_TX_C16)
#define MAIN_OPENART1_UART_RX          (UART4_RX_C17)

#define MAIN_TEST_LOG_UART             (UART_1)
#define MAIN_TEST_LOG_UART_TX          (UART1_TX_B12)
#define MAIN_TEST_LOG_UART_RX          (UART1_RX_B13)

/* 发车时小车中心点的网格坐标。Y=5.5 表示车心位于第 5 与第 6 行中心之间，
 * 运动距离计算必须保留这半格偏移，避免多走或少走 0.5 格。 */
#define MAIN_POS_NAV_START_X_GRID      (1.0f)
#define MAIN_POS_NAV_START_Y_GRID      (5.5f)

/* 将网格中心坐标转换到底盘使用的绝对米制坐标。
 * 坐标原点是可通行区域边界，不是发车点。 */
#define MAIN_POS_GRID_TO_M_X(g)        (((float)(g) - 0.5f) * CHASSIS_GRID_STEP_X_M)
#define MAIN_POS_GRID_TO_M_Y(g)        (((float)(g) - 0.5f) * CHASSIS_GRID_STEP_Y_M)

#define MAIN_POINT_NAV_WARMUP_TICKS    (200U)

#define MAIN_POS_NAV_TARGET_X_GRID     (14)
#define MAIN_POS_NAV_TARGET_Y_GRID     (10)
#define MAIN_POS_NAV_HOLD_YAW_DEG      (0.0f)
#define MAIN_POS_NAV_TARGET_X_M        MAIN_POS_GRID_TO_M_X(MAIN_POS_NAV_TARGET_X_GRID)
#define MAIN_POS_NAV_TARGET_Y_M        MAIN_POS_GRID_TO_M_Y(MAIN_POS_NAV_TARGET_Y_GRID)

#define MAIN_MENU_RENDER_DIV           (20U)

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_CONFIG_H_ */
