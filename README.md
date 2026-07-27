# 智能车视觉组推箱子工程

本仓库是第 21 届全国大学生智能汽车竞赛 AI 视觉组推箱子赛题的主控工程。系统以 NXP RT1064 为主控，使用四麦克纳姆轮底盘、IMU、编码器和 OpenART 视觉模块，在 16 x 12 虚拟地图上完成识图、分类、路径规划、推箱执行和状态管理。

工程主体是嵌入式 C 代码；`pc_sokoban/` 中保留了用于桌面验证推箱算法的 Python 工具。

## 目录结构

```text
visual-group/
├── README.md
├── libraries/                 # 逐飞 RT1064 SDK 与外设库
├── pc_sokoban/                # PC 端推箱算法验证、仿真与测试脚本
├── project/
│   ├── code/                  # 主控业务代码
│   │   ├── config/            # 引脚与底盘参数配置
│   │   │   ├── pinMap.h
│   │   │   └── configChassis.h
│   │   ├── chassis_*.c/.h     # 底盘驱动、里程计、运动学、PID、区域判定
│   │   ├── app_*.c/.h         # 视觉链路、识别流程、游戏状态机、位姿融合
│   │   └── algo_sokoban_solver.c/.h
│   ├── user/src/              # 主入口与中断入口
│   ├── iar/                   # IAR EWARM 工程
│   └── mdk/                   # MDK 工程
├── visual_inspection/         # 视觉相关素材或检查结果
└── *.md                       # 规则、方案、接口说明等辅助文档
```

## 主控代码架构

### 配置层

| 文件 | 作用 |
|------|------|
| `project/code/config/pinMap.h` | 电机、编码器、按键、OpenART UART 等硬件引脚映射 |
| `project/code/config/configChassis.h` | 底盘控制参数、速度限制、几何尺寸、视觉融合参数、方向极性 |
| `project/code/chassis_config.h` | 兼容入口，集中包含当前配置头并提供通用内联工具 |

### 底盘层

| 模块 | 作用 |
|------|------|
| `chassis_motor.*` | 电机 PWM 与方向控制 |
| `chassis_encoder.*` | 编码器读取与轮速反馈 |
| `chassis_imu.*` | IMU 航向角积分与滤波 |
| `chassis_mecanum.*` | 麦克纳姆轮正/逆运动学 |
| `chassis_pid.*` | 通用 PID 控制器 |
| `chassis_ctrl.*` | 底盘闭环、网格导航、位姿维护和到点判断 |
| `chassis_zone.*` | 发车区、边界、静止等几何区域判定 |
| `chassis_menu.*` | IPS 菜单与运行时参数管理 |

### 应用层

| 模块 | 作用 |
|------|------|
| `app_link.*` | OpenART 串口协议解析、地图/分类/位姿快照维护 |
| `app_game_logic.*` | 游戏主状态机，组织识别、规划、执行、部分匹配重读图和关卡切换 |
| `app_recognize.*` | 箱子与目标识别流程，生成完整或最大可行子集的类别映射 |
| `app_recognize_clear.*` | 识别路径被阻挡时的清障规划 |
| `app_vision_fusion.*` | OpenART 视觉位姿与底盘里程计融合 |
| `algo_sokoban_solver.*` | 推箱子求解、导航 BFS、死局判断和炸弹规划 |

### 入口层

| 文件 | 作用 |
|------|------|
| `project/user/src/main.c` | 系统初始化、运行模式选择、主循环调度 |
| `project/user/src/isr.c` | PIT、UART 等中断入口 |

## PC 验证工具

`pc_sokoban/` 用于在电脑端验证推箱算法逻辑：

| 文件 | 作用 |
|------|------|
| `sokoban_validator.py` | Python 版地图、BFS、死局与炸弹规划验证器 |
| `test_validator.py` | 验证器测试用例 |
| `test_partial_recognition_retry.py` | 第二/三关部分匹配优先执行与重读图回归 |
| `sokoban_car_sim.py` | 小车推箱仿真辅助脚本 |

## 工程入口

- IAR 工程：`project/iar/rt1064.eww`
- IAR 项目文件：`project/iar/program/rt1064.ewp`
- MDK 工程：`project/mdk/rt1064.uvprojx`
- 主入口：`project/user/src/main.c`

## 相关说明文档

仓库根目录还包含若干专题文档，例如规则摘要、视觉通信接口、视觉位姿融合、控制环参数方案和推箱子逻辑总结。这些文档用于补充具体设计背景；README 仅保留工程用途与文件架构概览。
