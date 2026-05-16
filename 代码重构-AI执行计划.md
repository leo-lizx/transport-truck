# 智能车视觉组主控工程 — 代码重构 AI 执行计划

> **文档用途**：指导人类或其他 AI 在 **不破坏现有功能与 P0 安全契约** 的前提下，对 `e:\car\visual-group` 主控 C 工程进行 **模块化拆分、注释补全、死代码清理**。  
> **必读上下文**：执行任何重构步骤前须通读 [`README.md`](./README.md)（尤其 §3–§4 模块职责、§16 协作铁律、调试索引）与 [`规则提炼.md`](./规则提炼.md)（赛题流程与算法边界）。  
> **工程事实快照**（以行数粗测为准，随仓库变化以实际为准）：`chassis_ctrl.c` ~1900 行、`algo_sokoban_solver.c` ~712、`chassis_menu.c` ~593、`app_game_logic.c` ~590、`app_link.c` ~497、`chassis_imu.c` ~410、`app_vision_fusion.c` ~404；其余底盘子模块较小。

---

## 1. 重构目标（必须达成）

| 目标 | 可验收标准 |
|------|------------|
| **控制与算法迭代可持续** | 单次需求变更时，典型 PR 主要触及 **1–3 个** `.c` 文件；单文件行数建议 **≤600 行**（硬上限 **≤900 行**，除非经评审说明无法拆分）。 |
| **结构可理解** | 每个业务 `.c` 文件顶部有 **模块头注释**（职责、调用节拍、依赖、线程/ISR 安全说明）；公共 API 在对应 `.h` 中有 **Doxygen 风格**简要说明。 |
| **便于调试** | 与 README「调试快速索引」一致：现象 → 文件映射 **不因拆分而丢失**；必要时在 `README.md` 中 **增补**新文件路径（仅当索引失效时）。 |
| **去除无用代码** | 经静态检查与链接验证：**无未引用 static、无重复逻辑大块、无永久 `#if 0` 垃圾堆**；调试宏保留须有 **开关宏 + 一句用途说明**。 |
| **禁止巨型单文件** | 新逻辑 **默认不得** 继续堆进已有超长 `.c`；应新建模块或子文件并更新 IAR 工程。 |

---

## 2. 硬约束与非目标（违反即视为失败）

### 2.1 硬约束

1. **并发与数据契约不变**（README §4、P0-3）：`g_game_map` / `app_link` 快照、`chassis_ctrl_get_pose()` seq-lock、`app_link_tick` 与主循环节拍关系 **不得因拆分而弱化**。  
2. **ISR 语义不变**：`isr.c` 内 5/10/20 ms 任务调用顺序、UART 喂字节路径 **只做等价迁移**，不在 ISR 内新增阻塞或长计算。  
3. **勿随意修改** `libraries/` 下逐飞 GPL 源码；业务代码保持在 `project/code/` 与 `project/user/src/`。  
4. **IDE**：新增/重命名 `.c` 必须同步 **`project/iar/program/rt1064.ewp`** 的 `code` 组；MDK 若使用需手动对齐。  
5. **命名与编码**：延续 README §16 — 底盘 `CHASSIS_`、链路 `APP_LINK_`/`PROTO_`、状态 `STAGE_`；源文件 **UTF-8 无 BOM**。  
6. **链接脚本**：`__size_cstack__` 等已针对 P0-4 调整，重构 **不得** 为「方便」回调小栈或把大数组改回栈上巨型对象。

### 2.2 非目标（本轮不做或低优先级）

- 不强制引入 RTOS、不重写整套控制律。  
- 不改变 OpenART 侧 `F:\main.py` 协议字节布局（除非单独立项并同步双端）。  
- 不在 `libraries/` 内做「风格统一」式大改。

---

## 3. 当前痛点与根因（供 AI 对齐认知）

1. **行数过长**：控制闭环、几何区域、导航、调试路径、姿态融合等混在 `chassis_ctrl.c`，算法与策略变更时 diff 面过大、冲突率高。  
2. **注释不足**：新人/AI 难以快速判断「这段代码在哪个节拍运行、能否 printf、是否持有锁」。  
3. **死代码累积**：历史调试分支、重复宏、未接 API（如 README 已知 TODO）与真实调用路径交织，增加阅读噪声。  
4. **单文件职责过多**：游戏状态机、链路、求解器虽已有独立文件，但 **底盘顶层** 与 **视觉融合** 仍偏臃肿。

---

## 4. 目标架构（逻辑分层）

下列分层 **只表达依赖方向**（上层可调用下层，反向仅通过回调/查询接口）：

```text
  main.c / isr.c          ← 入口与时基、硬件中断
       │
       ├── app_game_logic   ← 状态机、阶段、失败原因、与链路守护协作
       ├── app_link         ← 帧解析、地图权威副本、心跳统计
       ├── algo_sokoban_solver ← 推箱子与炸弹策略（纯算法，少碰硬件）
       │
       ├── app_vision_fusion（若保留）← 视觉侧与车体融合策略（不塞进 chassis_ctrl）
       │
       └── chassis_*        ← 底盘子系统
              ├── chassis_ctrl（薄门面：初始化、任务分发、对外 API 聚合）
              ├── chassis_odom / chassis_nav / chassis_zone（建议从 ctrl 拆出，见 §5）
              ├── chassis_imu / encoder / motor / mecanum / pid
              └── chassis_menu + chassis_config.h
```

**原则**：`chassis_ctrl` 将来应接近 **「编排 + 对外稳定 API」**，重算法与几何细节下沉到命名清晰的翻译单元。

---

## 5. 模块拆分建议（优先级排序）

> 下列拆分名为建议；实施时可微调文件名，但须在 **本文件 + README 调试索引** 中保持一致映射。

### P0（最高优先级）：拆解 `chassis_ctrl.c`

按 **节拍** 与 **问题域** 切分，减少跨文件循环依赖：

| 建议新文件（示例名） | 建议职责 | 典型调用方 |
|---------------------|----------|------------|
| `chassis_ctrl_core.c` | `init`、`task_5ms`/`task_20ms` 入口编排；对外 API 注册表；与 `chassis_menu` 调参交互的胶水代码 | `isr.c`、`main.c` |
| `chassis_odometry.c` + `.h` | 编码器积分、位姿更新、`pose` seq-lock 实现细节 | `chassis_ctrl_core` |
| `chassis_navigation.c` + `.h` | 点到点/网格目标、航向保持、速度指令合成（与地图格子、物理尺度相关） | `app_game_logic`、`chassis_ctrl_core` |
| `chassis_zone.c` + `.h` | 发车区、越界滞回、静止判定等 **几何区域**（README §6） | `app_game_logic` |
| `chassis_ctrl_debug.c` + `.h`（可选） | `MAIN_*` 调试模式、单轮 PID 打印、Live Watch 导出 | 仅调试编译路径 |

**验收**：`chassis_ctrl.h` **对外 API 保持兼容**（符号名、语义不变），或提供兼容宏/薄包装并 **一次性** 更新全工程引用且编译通过。

### P1：整理 `algo_sokoban_solver.c`

| 动作 | 说明 |
|------|------|
| 拆 `algo_sokoban_bfs.c` | 通用网格 BFS、队列、访问标记 |
| 拆 `algo_sokoban_bomb.c` | 炸弹策略与墙假设 |
| 拆 `algo_sokoban_deadlock.c` | 死局判定 |
| 保留 `algo_sokoban_solver.c` | 对外统一入口 `Sokoban_Solve_*`，包含赛题阶段调度 |

**验收**：桌面/车机对照路径不变；静态内存占用不明显劣化；README §7 表格更新指向新文件。

### P2：整理 `app_game_logic.c`

| 动作 | 说明 |
|------|------|
| 按 `STAGE_*` 拆多个 `app_game_stage_*.c` 或单文件内 **清晰分区 + static 阶段处理函数** | 若拆文件：每阶段一个 `static` 处理函数集合，公共上下文放入 `app_game_context.h`（仅数据结构，少逻辑） |
| 链路相关判断与 `app_link` 的交互 **不复制逻辑**，统一走 `app_link` 已有 API |

**验收**：状态转移图与 README §6 一致；链路超时/PAUSE 行为与 P0-2 一致。

### P3：整理 `app_link.c`

| 动作 | 说明 |
|------|------|
| 拆 `app_link_frame_parser.c` | 字节流状态机、CRC、LEN |
| 拆 `app_link_map_store.c` | 地图写入、seq-lock、`get_map_snapshot` |

**验收**：`g_link_stats` 字段语义不变；心跳/超时统计测试方法仍适用 README §10。

### P4：`chassis_menu.c` 与 `chassis_imu.c`

- 菜单：按「渲染 / Flash 存储 / 按键扫描」拆文件或至少 **分区注释 + 函数命名统一前缀**。  
- IMU：零偏、积分、滤波分区；头文件声明与 **5 ms 调用约束**写清楚。

### P5：`app_vision_fusion.c`

- 明确与 `chassis_ctrl_set_pose` 等 API 的边界；避免与 `chassis_odometry` 循环依赖；融合策略单独成层便于赛前调参。

---

## 6. 注释与文档规范（AI 生成代码时必须遵守）

### 6.1 每个 `.c` 文件顶部模板

```c
/*===========================================================================
 * [文件名] 一句话职责
 *
 * 调用节拍: 主循环 / 5ms ISR / 20ms ISR / 10ms ISR（择一或多选）
 * 线程安全: 是否使用 volatile / seq-lock / 仅单线程访问
 * 依赖模块: xxx.h, yyy.h
 * 对外 API: 见同名 .h
 *===========================================================================*/
```

### 6.2 函数注释（公共 API 必须在 `.h`）

- 参数单位（m、deg、grid index）与取值范围。  
- 是否可重入、是否可在 ISR 调用。  
- 失败或钳位行为（一句）。

### 6.3 中文说明

- 与 README §16 一致：**关键业务决策、状态机转移、赛题规则映射**用中文简注；避免废话式注释。

---

## 7. 死代码清理流程（必须可审计）

1. **静态侧**：使用 IDE 未引用查找、`grep` 交叉验证；对 `static` 函数优先。  
2. **链接侧**：IAR 全量链接后确认无 dead strip 异常（若启用）。  
3. **动态侧**：按 README §13 选最小 T0–T4 回归，再跑与改动相关的阶段。  
4. **删除原则**：  
   - 确认无调用、无文档引用、无菜单入口 → 删除。  
   - 仅赛题未接占位（如 `HAL_VISION_GET_BOX_CLASS_ID`）→ **保留单一占位 + TODO 编号 + README 对应节** 更新，不扩散复制。  
5. **禁止**：把大段旧代码改成 `#if 0` 留存；应删则删，版本控制在 Git。

---

## 8. 分阶段执行路线图（建议人类按 Sprint 采纳）

| 阶段 | 内容 | 退出标准 |
|------|------|----------|
| **S0 基线** | 打 tag；记录当前 `chassis_ctrl` 符号列表；跑通编译与最小上电测试 | 可复现二进制/或 map 文件存档 |
| **S1 只搬家不改逻辑** | `chassis_ctrl` 按 §5 拆文件，`static` 函数整体迁移；行为二进制尽量接近（允许地址变） | 全量编译 + T4 单轮 PID + T6 姿态相关测试通过 |
| **S2 补注释** | 新文件顶部 + 公共 API `.h` 注释补齐 | 代码审查清单清零 |
| **S3 死代码** | 按 §7 删除并跑链接 | map 体积不劣化或减小；无新 warning |
| **S4 算法文件** | P1 `algo_*` 拆分 | SokoPlayer/车机对照用例仍通过 |
| **S5 游戏/链路** | P2–P3 | 链路断连/恢复场景与 README 描述一致 |

每阶段结束：**更新 README 调试索引表**（仅当文件职责路径变化时）。

---

## 9. 与其他文档的同步义务

| 事件 | 必须更新的文档 |
|------|----------------|
| 新增/删除业务 `.c/.h` | `README.md` §3 目录树与「调试快速索引」表 |
| 影响赛题策略或阶段 | `规则提炼.md` 一般不随代码改；若规则理解变更则更新 |
| 跨会话任务 | `P0进度交接.md`（若仓库存在）记录重构进度与阻塞项 |

---

## 10. AI 执行清单（每次会话复制自检）

```
[ ] 已读 README.md §3–§4、§16 与本文 §2 硬约束
[ ] 已确认改动文件在 IAR ewp 的 code 组中
[ ] 未在 ISR 增加阻塞或 printf（除非既有调试点且默认关闭）
[ ] seq-lock / 地图快照 / pose 读取路径未被绕过
[ ] 单 PR 主题单一；附带「如何验证」3 条以内
[ ] 超长函数拆分优先于「行内小整理」
```

---

## 11. 风险与回滚

| 风险 | 缓解 |
|------|------|
| 拆分导致链接顺序/初始化次序变化 | `init` 仍从 `main.c` 单点调用；子模块仅提供 `*_init`，由 `chassis_ctrl_init` 编排顺序 |
| 头文件循环依赖 | 前向声明 + 把共享结构体抽到独立 `*_types.h` |
| 合并冲突 | S1 尽量「移动不修改」；避免同轮大面积格式化 |
| 栈/静态内存变化 | 大数组保持 `static` 或 `.bss` 习惯；不改为线程栈上大对象 |

回滚：Git 按阶段 tag 回退；禁止「半拆」长期留在主分支。

---

## 12. 验收总表（重构结束声明前逐项打勾）

- [ ] 核心业务 `.c` 无超过 **900 行**（或团队书面豁免）。  
- [ ] README 索引与真实路径一致。  
- [ ] IAR 全量编译 0 error，关键 warning 已处理或登记。  
- [ ] P0 链路、pose、地图快照 **行为与注释描述一致**。  
- [ ] 最小回归测试（README §13 子集）有记录。  
- [ ] 无已知大块死代码；调试代码均有宏开关。

---

## 12.5 执行进度（2026-05-13 本轮）

### 已落地（影响行为的物理改动）
- **chassis_zone 模块抽离** —— 从 `chassis_ctrl.c` 拆出 `chassis_zone.c/.h`：
  - 软限位常量 `SOFT_LIMIT_*`、helpers `zone_soft_limit_is_*`、入口 `apply_soft_limit_guard()` → `chassis_zone_apply_soft_limit_guard()`
  - 几何区域 API: `LaunchZone_e`、`chassis_zone_tick/is_in_launch/is_fully_outside_launch/is_out_of_bounds/clear_oob/is_static`
  - `chassis_body_speed_cmd_t` 加 struct tag `chassis_body_speed_cmd_s`，支持跨模块前向声明
  - `chassis_ctrl.h` 末尾 `#include "chassis_zone.h"` 保持源码兼容
  - IAR `rt1064.ewp` 已加入新文件
  - 行数变化：`chassis_ctrl.c` 2141 → 1851；新增 `chassis_zone.c` 305 / `chassis_zone.h` 111
- **README 调试索引同步** —— 拆出 chassis_zone 行

### 已落地（零逻辑变更：分区 banner 注释）
- `chassis_ctrl.c` —— 注入 §5~§10 章节标题（公共 API / 周期任务 / 运动指令 / 单轮 PID 调试 / 航向调试 / 状态查询）
- `chassis_imu.c` —— 注入 §1~§5 章节标题（硬件接口 / KF / 滑窗静止 / 初始化 / 5ms 链）
- `algo_sokoban_solver.c` —— 注入 §1~§7 章节标题（常量 / 导航 BFS / 观察点 / 单箱 BFS / Stage1-2 / 炸弹 / 航点）
- `app_link.c` —— 注入文件级章节索引（§1~§8）
- `app_game_logic.c` —— 注入 §1~§5 章节标题（执行上下文 / 航点公用 / 规划层 / Stage 处理 / 主入口）
- `chassis_menu.c` —— 注入 §1~§5 章节标题（参数读写 / Flash / 按键状态机 / UI 渲染 / 对外 API）
- `app_vision_fusion.c` —— 注入 §1~§3 章节标题（统计 / Snap / 一致性）

### 暂缓（保留至下一 sprint，理由已有依据）
- **chassis_ctrl_debug 抽离** —— 单轮 PID 调试与 attitude 调试函数与 `s_pid[]/s_mot[]/s_yaw_i/s_pos_i/s_v_along_lpf/s_v_cross_lpf/s_recovery_active/s_wheel_fb_lpf/s_ramp/s_last_cmd/s_pose seq-lock/s_mode/force_stop/pose_read_snapshot/stop_wheel_with_pid_reset` 等深度耦合，需新增 `chassis_ctrl_internal.h` 暴露大量私有状态，违背 P0「私有状态最小暴露」契约。本轮放弃，留待重新设计访问器层后单独 sprint。
- **algo_sokoban_solver 4 拆 / app_link 2 拆** —— 文件已在 900 行硬限内（868/559），ROI 不足；先以分区 banner 提升导航性，待 §12 验收前若仍需要再处理。
- **死代码清理 (S3)** —— `should_enter_next_level()` 是判分系统占位、所有 `#if` 块均有功能开关用途；本轮无明确死代码可删。

### 当前行数（本轮终态）
| 文件 | 行数 | 与 600 软目标 |
|---|---|---|
| `chassis_ctrl.c` | 1875 | 仍超（待 debug 抽离 sprint） |
| `chassis_zone.c` | 305 | OK |
| `chassis_zone.h` | 111 | OK |
| `chassis_imu.c` | 482 | OK |
| `algo_sokoban_solver.c` | 868 | 软目标超，硬限内 |
| `app_link.c` | 559 | OK |
| `app_game_logic.c` | 706 | 软目标超，硬限内 |
| `chassis_menu.c` | 689 | 软目标超，硬限内 |
| `app_vision_fusion.c` | 479 | OK |

> 后续 sprint 起点：先用本轮二进制做一次完整回归（T4 单轮 PID + T6 姿态 + T9 链路 + 现场最小推箱），再启动 `chassis_ctrl_debug` 抽离的访问器层设计评审。

---

## 13. 文档维护

- **维护人**：车队软件负责人。  
- **触发更新**：每完成 §8 中一整阶段，或模块边界再次调整时。  
- **版本**：2026-05-13 初版（与仓库当日结构对齐）。

---

**结语**：本计划刻意 **与 README / 规则提炼正交** — README 描述「是什么、怎么调」，本文描述「怎么拆、怎么收」。其他 AI 应 **先服从 README 铁律与本文件硬约束**，再发挥拆分与注释的具体实现。
