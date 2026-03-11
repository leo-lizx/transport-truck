#ifndef CHASSIS_MENU_H
#define CHASSIS_MENU_H

#include "chassis_config.h"

/*
 * 菜单模块公共接口：
 * - init: 系统启动时调用一次，加载当前底盘参数并初始化菜单状态。
 * - task_10ms: 10ms 周期调用，负责按键扫描与菜单状态机更新。
 * - render_100ms: 100ms 周期调用，负责屏幕刷新与参数显示。
 */
void chassis_menu_init(void);
void chassis_menu_task_10ms(void);
void chassis_menu_render_100ms(void);

#endif /* CHASSIS_MENU_H */
