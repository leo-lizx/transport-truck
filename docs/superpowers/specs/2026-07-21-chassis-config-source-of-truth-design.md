# 底盘参数单一真相源设计

## 目标

小车每次上电后，底盘运行时调参结构只从 `configChassis.h` 的编译期宏初始化，不再被菜单模块读取的历史 Flash 数据静默覆盖。

## 根因

`g_chassis_tune_params` 在 `chassis_ctrl.c` 中由 `configChassis.h` 初始化，`chassis_ctrl_init()` 对其进行安全限幅并配置轮速 PID。随后 `main.c` 调用 `chassis_menu_init()`；该函数初始化 Flash、读取合法参数块并调用 `chassis_ctrl_set_tune_params()`，因而旧 Flash 参数会覆盖刚初始化的配置值。

当前菜单模块没有对应的参数保存入口，Flash 代码只保留了历史参数加载行为。

## 方案

- 从 `chassis_menu.c` 移除调参 Flash 的初始化、读取、校验和覆盖逻辑。
- 保留 IPS200 菜单初始化、按键扫描和渲染行为。
- 保留 `chassis_ctrl_get_tune_params()` 与 `chassis_ctrl_set_tune_params()`，不改变运行中临时调参接口。
- 修正 `configChassis.h` 中持久化语义说明，明确编译期宏是上电参数唯一来源。

不修改控制算法、参数数值、Flash 驱动和 PC 推箱子镜像。

## 验证

- 静态回归测试应证明菜单初始化路径不再包含 `flash_init()`、`flash_read_page()` 或 `chassis_ctrl_set_tune_params()`。
- 检查 `g_chassis_tune_params` 的所有初始化和写入入口，确认正常启动路径只保留配置初始化与安全限幅。
- 运行项目现有底盘配置和到位相关测试。
- IAR 实机验证：修改一个易观察的配置参数并重新编译烧录，上电后读取运行时参数，确认与安全限幅后的配置值一致。

## 资源与实时性

该修改删除代码，不新增 BSS；移除一次启动期 Flash 初始化和读取，不影响 5ms/20ms 控制中断路径。
