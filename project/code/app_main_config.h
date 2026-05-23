/*===========================================================================
 * [app_main_config.h] 主程序运行模式与调试参数配置
 *
 * 本文件集中放置 main 入口相关的编译期选项、串口选择、导航起点/目标点、
 * 单轮 PID 调试参数等。业务代码通过这些宏裁剪不同运行模式。
 *===========================================================================*/

#ifndef APP_MAIN_CONFIG_H_
#define APP_MAIN_CONFIG_H_

#include "zf_common_headfile.h"
#include "chassis_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 顶层运行模式选择:
 * 这些模式通过编译期宏裁剪，只会编译 MAIN_RUN_MODE 选中的那一条主流程。
 */
#define MAIN_RUN_MODE_GAME             (0)
#define MAIN_RUN_MODE_YAW_HOLD         (1)
#define MAIN_RUN_MODE_SINGLE_WHEEL     (2)
#define MAIN_RUN_MODE_POINT_NAV        (3)
#define MAIN_RUN_MODE_SOKO_SELFTEST    (4)
#define MAIN_RUN_MODE_OPENART1_TEST    (5)
#define MAIN_RUN_MODE_STATIC_MAP_DRIVE (6)

/* 修改 MAIN_RUN_MODE 即可切换整车顶层任务。 */
#ifndef MAIN_RUN_MODE
#define MAIN_RUN_MODE                  (MAIN_RUN_MODE_STATIC_MAP_DRIVE)
#endif

/* 静态地图来源:
 *   0 = 从 OpenART1 的 UART4 MAP 帧接收地图
 *   1 = 使用 app_main_modes.c 内置的手写地图，方便无视觉端时联调
 */
#ifndef SMD_USE_MANUAL_MAP
#define SMD_USE_MANUAL_MAP             (1)
#endif

/* OpenART1 视觉端通信串口配置。 */
#define MAIN_OPENART1_UART             (UART_4)
#define MAIN_OPENART1_UART_TX          (UART4_TX_C16)
#define MAIN_OPENART1_UART_RX          (UART4_RX_C17)

/* 测试日志串口配置，常用于自测和 OpenART1 链路显示。 */
#define MAIN_TEST_LOG_UART             (UART_1)
#define MAIN_TEST_LOG_UART_TX          (UART1_TX_B12)
#define MAIN_TEST_LOG_UART_RX          (UART1_RX_B13)

/* 车体初始网格位置，支持 0.5 这种“位于两格之间”的物理中心描述。 */
#define MAIN_POS_NAV_START_X_GRID      (1.0f)
#define MAIN_POS_NAV_START_Y_GRID      (5.5f)

/* 将网格坐标转换为 chassis_ctrl 使用的米制坐标。 */
#define MAIN_POS_GRID_TO_M_X(g)        (((float)(g) - 0.5f) * CHASSIS_GRID_STEP_X_M)
#define MAIN_POS_GRID_TO_M_Y(g)        (((float)(g) - 0.5f) * CHASSIS_GRID_STEP_Y_M)

/* 上电后等待底盘/传感器稳定的调度 tick 数。 */
#define MAIN_POINT_NAV_WARMUP_TICKS    (200U)
#define MAIN_HOLD_YAW_TARGET_DEG       (0.0f)

/* POINT_NAV 调试模式的默认目标点。 */
#define MAIN_POS_NAV_TARGET_X_GRID     (14)
#define MAIN_POS_NAV_TARGET_Y_GRID     (10)
#define MAIN_POS_NAV_HOLD_YAW_DEG      (0.0f)
#define MAIN_POS_NAV_TARGET_X_M        MAIN_POS_GRID_TO_M_X(MAIN_POS_NAV_TARGET_X_GRID)
#define MAIN_POS_NAV_TARGET_Y_M        MAIN_POS_GRID_TO_M_Y(MAIN_POS_NAV_TARGET_Y_GRID)

/* SINGLE_WHEEL 模式下的单轮 PID 调试参数。 */
#define MAIN_PID_DEBUG_WHEEL_INDEX     (CHASSIS_WHEEL_LF)
#define MAIN_PID_DEBUG_TARGET_MPS      (3.3f)
#define MAIN_PID_DEBUG_FORCE_PID       (1)
#define MAIN_PID_DEBUG_KP              (40.0f)
#define MAIN_PID_DEBUG_KI              (23.0f)
#define MAIN_PID_DEBUG_KD              (0.0f)

/* 菜单刷新分频，主循环按 5ms tick 调度时 20 次约等于 100ms。 */
#define MAIN_MENU_RENDER_DIV           (20U)

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_CONFIG_H_ */
