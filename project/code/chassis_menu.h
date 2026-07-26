#ifndef CHASSIS_MENU_H
#define CHASSIS_MENU_H

#include "chassis_config.h"

/* 1=启用调试菜单生命周期（IPS/按键/Flash 参数加载/周期扫描与渲染）。
 * 设为 0 可在无屏幕模式下排查启动问题。 */
#ifndef CHASSIS_MENU_ENABLE
#define CHASSIS_MENU_ENABLE (1)
#endif

/*
 * 菜单模块公共接口：
 * - init: 系统启动时调用一次，加载当前底盘参数并初始化菜单状态。
 * - task_10ms: 10ms 周期调用，负责按键扫描与菜单状态机更新。
 * - render_100ms: 100ms 周期调用，负责屏幕刷新与参数显示。
 * - render_game_frozen_100ms: 比赛模式仅在新的冻结地图产生时更新地图。
 */
void chassis_menu_init(void);
void chassis_menu_task_10ms(void);
void chassis_menu_render_100ms(void);
void chassis_menu_render_game_frozen_100ms(void);

#endif /* CHASSIS_MENU_H */
