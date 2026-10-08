# transport-truck — RT1064 运输车主控基础工程

> 保留底盘控制基础；运输业务和新菜单后续构建
>
> 本文档面向工具链和 AI 编码助手，提供可执行参考信息。
> 模块列表和目录结构见 [README.md](README.md)。

## 硬件

| 项目 | 规格 |
|------|------|
| MCU | NXP RT1064DVL6A, Cortex-M7 @ 600MHz |
| 底盘 | 4 Mecanum 轮, DRV8701E 双路电机驱动 |
| 编码器 | 4 路正交编码器 (QTIMER1: LB+RB, QTIMER2: LF+RF) |
| IMU | IMU660RB (SPI4, SCK=C23 MOSI=C22 MISO=C21 CS=C20) |
| 显示屏 | IPS200（驱动保留，旧菜单已移除） (SPI3, SCK=B0 MOSI=B1 RST=B2 DC=C19 CS=B3 BL=C18) |
| 按键 | K1~K4 (C15~C12) |
| 预留串口 | LPUART4 (C16/C17) + LPUART1 (B12/B13)，保留 app_link 的既有协议 |
| 调试串口 | 115200bps 8N1, VOFA+ FireWater 协议 |

## 编译

```bash
# 主工具链: IAR EWARM 9.2
# 打开工作空间
project/iar/rt1064.eww

# 活动配置
nor_sdram_zf_dtcm

# 编译
Project → Rebuild All  (或 Ctrl+F7 单文件编译)
```

- 备选工具链: MDK/Keil (`project/mdk/rt1064.uvprojx`)
- C 标准: C11, 编译选项 `--c99`
- 编码: UTF-8 no BOM, 4 空格缩进

## 烧录与调试

| 操作 | 方法 |
|------|------|
| 调试器 | CMSIS-DAP / J-Link |
| 烧录 | IAR → Download and Debug (Ctrl+D) |
| 串口监控 | 115200bps 8N1, VOFA+ FireWater 12 通道 |
| Live Watch | IAR 调试视图，观测 `s_main_tick_*` / `g_chassis_*` 全局变量 |

## 硬件契约层

| 文件 | 职责 |
|------|------|
| [project/code/config/pinMap.h](project/code/config/pinMap.h) | 零依赖纯 `#define` 引脚映射 |
| [project/code/config/configChassis.h](project/code/config/configChassis.h) | 分节参数 §A~§F，含 **§极性段**（所有方向系数的 SSOT） |
| [project/code/chassis_config.h](project/code/chassis_config.h) | 聚合入口头 (include pinMap + configChassis + 枚举 + inline 工具) |

> **极性 SSOT 规则**: 所有方向系数 (`*_OUTPUT_DIR`, `ENCODER_*_DIR`, `IMU_YAW_SIGN`) 集中在 `configChassis.h` §极性段。PID Kp 全部为正；符号由极性段统一处理，不散落在各模块。

## 关键入口

| 文件 | 作用 |
|------|------|
| [project/user/src/main.c](project/user/src/main.c) | 系统初始化、主循环调度、5ms tick 节拍 |
| [project/user/src/isr.c](project/user/src/isr.c) | PIT/UART 中断入口 |
| [project/code/chassis_ctrl.c](project/code/chassis_ctrl.c) | 底盘顶层控制 (IMU→里程计→导航→运动学→PID→电机) |
| [project/code/app_link.c](project/code/app_link.c) | 视觉串口帧协议解析 (CRC8/状态机/Seq-Lock) |

## 硬约束

| 约束 | 值 | 来源 |
|------|-----|------|
| SRAM 预算 | ≤ 512KB | CLAUDE.md |
| 新增 BSS 数组上限 | ≤ 10KB | CLAUDE.md |
| 姿态采样 / 底盘闭环 | 5ms / 20ms | main.c / isr.c |

## 文档索引

| 文档 | 内容 |
|------|------|
| [README.md](README.md) | 工程概览、目录结构、模块说明 |
| [代码编写与变更规范.md](代码编写与变更规范.md) | 编码规范、审查清单、AI 助手强制指令 |
| [.hecateflow/project.json](.hecateflow/project.json) | HecateFlow 项目清单 (外设/约束/构建系统) |
