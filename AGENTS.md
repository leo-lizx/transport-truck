# AGENTS.md — 推箱子视觉小车项目

> 本文件与 `CLAUDE.md` 内容镜像一致。
> 跨 CLI 最大公约数入口 (Claude Code / OpenCode / Codex)。

## 强制规则

### 代码编写前必须阅读规范

在进行**任何代码修改、编写、重构**之前，必须先阅读：

**`代码编写与变更规范.md`** — 代码编写与变更的强制性规范，包含：
- 规则优先级（用户需求 > 项目接口 > 模块惯例 > 通用规则 > 个人偏好）
- 核心原则：先理解后修改、只实现已确认需要的功能（YAGNI）、最小必要改动、简单优先
- 模块化设计、编码可读性、注释与文档规范
- 兼容性与安全边界、测试与验证要求
- 标准工作流程（修改前/中/后）
- 功能必要性决策表、代码审查清单
- **AI 编码助手强制指令（第 11 节）**

### 修改前检查清单

- [ ] 已阅读 `代码编写与变更规范.md` 第 11 节（AI 强制指令）
- [ ] 已评估对 BSS 内存预算和 5ms tick 实时约束的影响
- [ ] 已确认是否需要同步更新 PC 镜像 `pc_sokoban/sokoban_validator.py`

## 项目结构

```
visual-group/
├── project/code/           # 固件 C 代码 (RT1064)
│   ├── config/                  # 硬件契约层 (pinMap + configChassis)
│   ├── algo_sokoban_solver.c/h  # 推箱子求解器 (导航+推箱+炸弹+后处理)
│   ├── app_game_logic.c/h       # 游戏主状态机
│   ├── app_recognize.c/h        # 地图识别 Tour 子状态机
│   ├── app_recognize_clear.c/h  # 清障导航
│   ├── app_vision_fusion.c/h    # 视觉位姿融合
│   └── app_link.c/h             # 通信链路
├── pc_sokoban/             # PC 端 Python 镜像/验证器
│   ├── sokoban_validator.py     # 核心算法镜像
│   └── test_validator.py        # 回归测试
└── project/iar/            # IAR 工程文件
```

## 关键约束

- **MCU**: NXP RT1064 (Cortex-M7)
- **SRAM 预算**: ~512KB (DTCM + OCRAM)，BFS static 数组约 35KB，禁止新增 >10KB 的 BSS 数组
- **实时约束**: `Game_Logic_Task_Run` 单次执行 < 800μs（5ms tick 留余量）
- **不可重入**: `algo_sokoban_solver` 内部函数共享 static buffer，禁止 ISR/嵌套调用
- **C/Python 同步**: 算法修改必须在两端同步

## 相关文档

- [PROJECT.md](PROJECT.md) — 编译/烧录/调试/硬件契约层入口
- [代码编写与变更规范.md](代码编写与变更规范.md) — 强制编码规范
- [规则提炼.md](规则提炼.md) — 赛规摘要
- [README.md](README.md) — 完整目录结构与模块说明
- [.hecateflow/project.json](.hecateflow/project.json) — HecateFlow 项目清单
