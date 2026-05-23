/*===========================================================================
 * [app_main_modes.h] 主程序模式分发表接口
 *
 * main.c 只需要调用本文件暴露的初始化、底盘后处理、5ms tick 和菜单判断接口；
 * 具体运行模式由 app_main_modes.c 根据 MAIN_RUN_MODE 编译期选择。
 *===========================================================================*/

#ifndef APP_MAIN_MODES_H_
#define APP_MAIN_MODES_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 函数: App_MainModes_InitCommunication
 * 功能: 初始化当前运行模式需要的通信外设，例如 OpenART1 串口或测试日志串口。
 * 调用: main 初始化阶段调用一次，通常早于底盘业务启动。
 */
void App_MainModes_InitCommunication(void);

/*
 * 函数: App_MainModes_AfterChassisInit
 * 功能: 底盘初始化完成后调用，用于启动当前模式的初始控制目标或测试界面。
 * 调用: chassis_ctrl_init() 等底盘模块初始化完成后调用一次。
 */
void App_MainModes_AfterChassisInit(void);

/*
 * 函数: App_MainModes_Task5ms
 * 功能: 主循环 5ms 调度入口，根据 MAIN_RUN_MODE 分发到具体任务。
 * 约束: 函数内不应做无限等待，避免阻塞下一次 PIT tick 调度。
 */
void App_MainModes_Task5ms(void);

/*
 * 函数: App_MainModes_ShouldRenderMenu
 * 功能: 返回当前模式是否需要刷新 IPS 菜单。部分测试模式会独占屏幕。
 * 返回: 1=允许菜单渲染；0=由当前模式自行管理屏幕。
 */
uint8 App_MainModes_ShouldRenderMenu(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_MODES_H_ */
