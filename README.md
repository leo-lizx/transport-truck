# 智能车视觉组推箱子工程

第 21 届全国大学生智能汽车竞赛 · AI 视觉组 · 推箱子赛题的主控工程。系统以 NXP RT1064 为主控，配合四麦克纳姆轮底盘、IMU、编码器与 OpenART 视觉模块，在 16×12 虚拟地图上完成识图分类、路径规划、推箱执行与关卡状态管理。

> 最终成绩：东北赛区省二等奖。

工程主体是嵌入式 C 代码；`pc_sokoban/` 保留桌面端验证推箱算法的 Python 工具。

## 目录结构

```text
visual-group/
├── project/                 # 固件 C 代码 (RT1064)
│   ├── code/                # 主控业务代码（新增代码统一放这里，勿建子文件夹）
│   │   ├── config/          #   引脚映射 + 底盘参数 + 方向极性（SSOT）
│   │   ├── chassis_*.c/.h   #   底盘层：电机/编码器/IMU/麦克纳姆/PID/闭环/区域
│   │   ├── app_*.c/.h       #   应用层：通信/游戏状态机/识别/清障/位姿融合
│   │   └── algo_*.c/.h      #   推箱子求解：导航 BFS + 推箱 + 炸弹 + 后处理
│   ├── user/                # 主入口 main.c 与中断入口 isr.c
│   ├── iar/                 # IAR EWARM 工程
│   └── mdk/                 # MDK/Keil 工程
├── pc_sokoban/              # PC 端推箱算法验证器 / 仿真 / 回归测试 (Python)
├── libraries/               # 逐飞 RT1064 SDK 与外设驱动（第三方，勿改）
├── visual_inspection/       # OpenART 视觉模型 (tflite) 与检查脚本
├── docs/                    # PINOUT 引脚冲突矩阵等
├── 修改日志/                # 按日期的修改简报（算法/流程变更记录）
└── memories/                # 方案背景与决策记录
```

## 主控代码分层

| 层级 | 前缀 | 作用 |
|------|------|------|
| 配置 | `config/` | 引脚映射 `pinMap.h`、底盘参数与极性 `configChassis.h`、兼容入口 `chassis_config.h` |
| 底盘 | `chassis_*` | `motor`/`encoder`/`imu`/`mecanum`/`pid`/`ctrl`（闭环+导航）/`zone`（区域判定）/`menu` |
| 应用 | `app_*` | `link`（OpenART 串口协议）/`game_logic`（主状态机）/`recognize`（识别）/`recognize_clear`（清障）/`vision_fusion`（位姿融合） |
| 算法 | `algo_*` | `algo_sokoban_solver`：求解 + BFS 导航 + 死局判断 + 炸弹规划 |
| 入口 | — | `project/user/src/main.c`（初始化 + 模式选择 + 主循环）、`isr.c`（PIT/UART 中断） |

## 规则与文档放置

| 文档 | 位置 | 内容 |
|------|------|------|
| 比赛规则提炼 | `规则提炼.md` | 赛规与比赛流程的提炼，供算法开发与现场策略参考 |
| 代码编写规范 | `代码编写与变更规范.md` | 代码编写/变更的强制规范（含 AI 助手强制指令） |
| 自动注入规则 | `.claude/rules/` | `c-style.md`（C 编码风格）、`embedded-safety.md`（嵌入式安全）；触发关系见其 `README.md` |
| 工具链/助手入口 | `CLAUDE.md`、`AGENTS.md`、`PROJECT.md` | 面向 AI 编码助手与工具链的工程说明（`AGENTS.md` 为跨 CLI 镜像） |
| 视觉通信接口 | `视觉通信接口说明.md` | 给视觉端同学：接线、串口参数、帧与地图 payload 格式 |
| 视觉位姿融合 | `视觉位姿融合实现说明.md` | OpenART 格坐标 + 主控里程计的融合实现 |
| 底盘控制评审 | `底盘运动控制角度环与位置环评审及最优方案.md` | 角度环/位置环方案评审与结论 |
| 引脚冲突矩阵 | `docs/PINOUT.md` | 由 `pinMap.h` 自动提取，新增外设时查冲突 |
| 修改日志 | `修改日志/` | 按日期的修改简报 |
| 方案决策记录 | `memories/session/plan.md` | 视觉位姿融合等方案背景与决策 |

## PC 验证工具

`pc_sokoban/` 用于在桌面端验证推箱算法，修改算法后需在 C/Python 两端同步：

| 文件 | 作用 |
|------|------|
| `sokoban_validator.py` | 地图 / BFS / 死局 / 炸弹规划的 Python 镜像验证器 |
| `test_validator.py` | 验证器回归测试 |
| `test_partial_recognition_retry.py` | 第二/三关部分匹配优先执行与重读图回归 |
| `sokoban_car_sim.py`、`test_car_sim_line_sweep.py` | 推箱仿真与辅助测试 |

## 工程入口

- IAR 工程：`project/iar/rt1064.eww`
- MDK 工程：`project/mdk/rt1064.uvprojx`
- 主入口：`project/user/src/main.c`
