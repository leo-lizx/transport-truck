# 视觉位姿融合方案（OpenART 整数格坐标）

## 算法选型决策（2026-05 重构）

**采用：事件驱动 Snap-to-Grid + 运动中一致性监控**

| 候选 | 易实现 | 准确度 | 与本工程契合度 | 决策 |
|------|--------|--------|----------------|------|
| 连续互补滤波（alpha） | 已实现 | 受 ±半格量化牵引 | 中 | 保留作可选（默认关） |
| EKF / UKF | 复杂 | 不优于互补（单一量化观测） | 低 | 不采用 |
| 粒子滤波 | 很复杂 | 高，但杀鸡用牛刀 | 低 | 不采用 |
| **事件驱动 Snap + 一致性监控** | 简单 | 高（终点 ≈ 格中心物理标定精度） | 高 | **采用** |

### 选型理由

- OpenART 整数格 16×12 → 单元约 0.23m，单次观测量化误差 **±0.115m**，远大于短距 odom 漂移（≈3cm）。
- 运动中持续融合 = 把高方差观测注入低方差状态，**短行程是负收益**。
- 推箱子精度敏感时刻只在「到达推送格 / 终点格」一拍：**事件驱动 Snap** 把视觉的「整数格识别」用得最足。
- 大幅打滑 / 被搬动 = 单点位置异常 → 用 **一致性监控** 兜底，避免基于错位姿规划。

## 推荐结构

- 编码器 + IMU 高频积分 → 平滑、实时的短时位姿。
- OpenART 整数格 → 离散 Anchor，在「到位 + 短静止」时做多帧表决并 Snap 到格中心。
- 主循环侧新增三个 hook：
  - `app_vision_fusion_task`（运动中连续融合，默认空）
  - `app_vision_fusion_consistency_tick`（运动中一致性监控）
  - `app_vision_fusion_snap_request` / `_state` / `_cancel`（航点闸门）

## 核心流程

1. OpenART 发送 MAP+车位（LEN=194），主控 `app_link_get_car_snapshot()` 提供 seq-lock 快照（含 `frame_id`）。
2. 网格坐标转物理坐标走 `chassis_grid_x_to_m / _y_to_m`（已含内场钳位）。
3. **到站 Snap**：`is_arrived` 后短静止门 (`SETTLE_MS`) → 同 `frame_id` 去重投票 → ≥ `VOTE_MIN` 且与目标差 ≤ `MAX_GAP_CELLS` 即 `chassis_ctrl_set_pose(grid_to_m, yaw)`；超时/REJECT 也终态放行。
4. **一致性监控**：每 `PERIOD_MS` 比较 `|odom 格 - 视觉格|`；持续 `HOLD_MS` ≥ `GAP_CELLS` 触发硬重定位 + 冷却。
5. **可选连续融合**（默认关）：误差分级 + 限幅 + 延迟补偿，仅在视觉端升级到亚格坐标后再启用。

## 源码入口

| 内容 | 位置 |
|------|------|
| 融合开关与参数 | `project/code/chassis_config.h`（`CHASSIS_VISION_*`） |
| 车辆坐标快照 | `app_link_get_car_snapshot()`，`app_link.c` |
| 全局速度（延迟补偿） | `chassis_ctrl_get_odom_velocity_global_mps()` |
| 导航目标（Snap 校验） | `chassis_ctrl_get_point_nav_target_m()` |
| Snap 状态机 | `app_vision_fusion_snap_*`，`app_vision_fusion.c` |
| 一致性监控 | `app_vision_fusion_consistency_tick()`，`app_vision_fusion.c` |
| 航点闸门 | `chassis_nav_arrived_for_waypoint()`，`app_game_logic.c` |
| 融合 / 监控调用 | `Game_Logic_Task_Run()`，`app_game_logic.c` |
| IAR 工程 | `project/iar/program/rt1064.ewp` 已注册 |

默认开关：`CHASSIS_VISION_SNAP_ON_ARRIVE_ENABLE=1`、`CHASSIS_VISION_CONSISTENCY_ENABLE=1`、`CHASSIS_VISION_FUSION_ENABLE=0`。

## 实车验证顺序

1. 关 Snap & 一致性，跑完整推箱：记录 odom 格 / 视觉格 / 误差，作为基线。
2. 开 Snap：看终点对齐；`snap_timeout / snap_reject` 应远小于 `snap_done`。
3. 开一致性监控：故意搬一下车，应在 ≈ `HOLD_MS` 内自我重定位（`consistency_fire++`）。
4. 长跑无人为扰动 5 分钟：`consistency_fire` 应保持 0。
5. 异常：遮挡 / 断串口 / 打滑 / 搬车 — 确认丢弃陈旧帧或回退到 odom 兜底，不卡死航点。

## 视觉端后续建议

整数格只能做纠偏与到站确认；要厘米级闭环则在 `main.py` 输出亚格坐标（×10/×100，或物理米 `int16`）并随帧携带置信度/帧号；相应扩展 `app_link` 协议长度，同时再考虑启用连续融合或单维 1D KF。
