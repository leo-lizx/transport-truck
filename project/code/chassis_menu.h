#ifndef CHASSIS_MENU_H
#define CHASSIS_MENU_H

#include "chassis_config.h"

/**
 * @brief  初始化菜单模块。
 *         从 Flash 或内置默认值加载当前底盘调参（chassis_config_get()），
 *         并初始化内部菜单状态机与 IPS 屏幕显示。
 * @note   调用时机：main.c 中 chassis_ctrl_init() 完成后调用一次。
 *         禁止在中断中调用。
 */
void chassis_menu_init(void);

/**
 * @brief  菜单按键扫描与状态机驱动（10ms 周期任务）。
 *         扫描 IPS 旁按键、切换菜单光标、修改参数值，
 *         按下保存键后调用 chassis_config_save_to_flash() 持久化。
 *         修改的参数通过 chassis_config_apply() 立即写入活动参数，实时生效。
 * @note   调用时机：main.c 主循环每 10ms 调用一次。
 *         禁止在中断中调用；Flash 写入操作耗时约 1~5ms，只在按键确认时触发。
 */
void chassis_menu_task_10ms(void);

/**
 * @brief  IPS 屏幕参数显示刷新（100ms 周期任务）。
 *         将当前活动调参值、菜单光标位置、当前模式等信息渲染到 IPS 屏幕。
 *         与 task_10ms 分频解耦：屏幕刷新比按键扫描慢，避免频繁 SPI 传输。
 * @note   调用时机：main.c 主循环通过 MAIN_MENU_RENDER_DIV 分频，每约 100ms 调用一次。
 *         禁止在中断中调用。
 */
void chassis_menu_render_100ms(void);

#endif /* CHASSIS_MENU_H */
