# C 编码风格规则

> 本文档是 HecateFlow `references/embedded-c-style.md` 的项目本地版本。
> 适用: RT1064 C11 裸机固件 (IAR EWARM 9.2 + MDK/Keil)。

## 文件编码

- 源文件统一 **UTF-8 (无 BOM)**
- 行尾 LF，缩进 4 空格（不用 Tab）
- 禁止在 `.icf`/`.ld` 链接脚本中写中文或非 ASCII 注释（IAR ILINK 会崩溃）

## 数据类型

- 用固定宽度类型：`uint8_t` / `int16_t` / `uint32_t` / `float`
- 禁止裸 `int` / `unsigned`（平台宽度歧义）
- 布尔用 `uint8_t` (`0u` / `1u`)，除非已统一 `stdbool.h`
- 浮点字面量带 `f` 后缀：`0.0f`、`-18.0f`

## 路径纪律

- 构建配置、`#include`、LSP `-I`、脚本一律**相对路径**，禁止绝对机器路径
- `#include` 用相对头路径时，对应目录必须已在构建系统 include 搜索路径中

## 条件编译

- `#if`/`#ifdef` 内的表达式不要有副作用
- 多层模式宏守卫必须对齐：函数定义守卫 / 调用点守卫 / ISR 路由守卫 三处一致
- 删掉无用 `#if 0` 块——版本控制已保存历史

## 命名

- 模块级函数加模块前缀：`chassis_*`, `app_*`, `algo_*`
- 宏全大写：`CHASSIS_POS_KP`；枚举值按模块：`CHASSIS_WHEEL_LF`
- 单位放入名称或注释：`_mps` (m/s)、`_dps` (°/s)、`_deg` (°)、`_us` (µs)

## inline

- 头文件中的 `inline` 函数必须加 `static`，否则多编译单元链接报错
- 非共享的辅助函数放 `.c` 文件，不用 inline 暴露到头文件
