# 智能车视觉组 — 推箱子主控工程

> **平台**: NXP RT1064DVL6A (Cortex-M7 @ 600MHz)
> **比赛**: 第 21 届全国大学生智能汽车竞赛 · AI 视觉组（推箱子赛题）
> **底盘**: 4 麦克纳姆轮 + DRV8701E 双路驱动 + IMU660RB
> **视觉**: OpenART Plus 副镜头（UART 115200，CRC8 协议帧）
> **最近更新**: 2026-04-24（P0-8 完成，P0-6 进行中）

---

## 1. 工程速览

```text
visual-group/
├── README.md              ← 本文件
├── P0进度交接.md           ← 当前任务进度 / 跨会话上下文（必读）
├── 规则提炼.md             ← 比赛规则与策略
├── 测试方案.md             ← 联调测试用例
├── 工程讲解与调试手册.md    ← 详细模块讲解
├── libraries/             ← 逐飞 SDK + zf_common/device/driver
└── project/
    ├── code/              ← 用户代码（平铺 .c/.h）
    ├── user/src/          ← main.c / isr.c
    ├── iar/               ← IAR EWARM 工程（主用）
    └── mdk/               ← MDK5 工程（仅占位，未维护）
```

### `project/code/` 模块清单

| 模块 | 文件 | 职责 |
|---|---|---|
| 统一配置 | `chassis_config.h` | 引脚 / 物理尺寸 / PID / 限幅 / 几何（改硬件只改这里） |
| PID | `chassis_pid.c/.h` | 通用增量式 PID |
| 电机 | `chassis_motor.c/.h` | DRV8701E PWM + DIR |
| 编码器 | `chassis_encoder.c/.h` | 正交脉冲 → 轮速 |
| IMU | `chassis_imu.c/.h` | IMU660RB SPI，零偏标定 + 航向积分 |
| 麦轮运动学 | `chassis_mecanum.c/.h` | 正/逆运动学 + 等比例限速 |
| 顶层底盘 | `chassis_ctrl.c/.h` | 5/20ms 闭环 + 网格定点 + 航向保持 + **几何区域判定** + `chassis_ctrl_move_to_grid()` 对外 API |
| 调参菜单 | `chassis_menu.c/.h` | 2 寸 IPS + 4 按键 + Flash 持久化 |
| 推箱求解 | `algo_sokoban_solver.c/.h` | 地图分解法 + 单箱位图 BFS + 炸弹策略 + 导航 BFS（合并原 `algo_bfs_scout`）|
| 游戏状态机 | `app_game_logic.c/.h` | 7+1 态主流程 + 链路守护 |
| **链路协议** | `app_link.c/.h` | CRC8 帧解析 + seq-lock 地图快照 + 心跳超时 |

---

## 2. 系统架构

```text
                 ┌─────────────────────────────────────────────┐
   OpenART ──UART1──► app_link.c (CRC8 解析 + seq-lock 写入)    │
                              │                                │
                              ▼                                │
                     权威地图副本 s_map_authoritative           │
                              │                                │
   主循环 (wait_for_tick + __WFI, 5ms PIT 唤醒)                 │
   ┌──────────────────────────┼────────────────────────────┐   │
   │ Game_Logic_Task_Run()    │                            │   │
   │  ① update_link_state     │ ② chassis_zone_tick        │   │
   │  ③ check_out_of_bounds   │ ④ app_link_get_map_snapshot│   │
   │  ⑤ switch(current_stage) │                            │   │
   └──────────────────────────┼────────────────────────────┘   │
                              ▼                                │
                  chassis_ctrl_move_to_grid()                 │
                              │                                │
   PIT_CH0 (5ms): IMU 采样 + 航向积分 + 链路 tick               │
   PIT_CH1 (20ms): 编码器 → 里程计 → 导航 P → 麦轮逆 → PID → PWM│
   PIT_CH2 (10ms): 菜单按键扫描                                 │
   └──────────────────────────────────────────────────────────┘
```

### 关键并发安全（P0-3）
- **`g_game_map`**: `app_link.c` 内部权威副本 `s_map_authoritative` + `s_map_seq` seq-lock；主循环每帧 `app_link_get_map_snapshot()` 读到稳定快照后再用。
- **`s_pose`**: `chassis_ctrl.c` 内 yaw（5ms ISR 写）/ x,y（20ms ISR 写）/ 主循环写均走 `pose_write_begin/end`；外部一律 `chassis_ctrl_get_pose()` 读快照。
- 重试放弃计数 `g_link_map_snapshot_retry_giveup` / `g_chassis_pose_snapshot_retry_giveup` 长期应保持 0。

### 主循环节拍（P0-5）
- 主循环改 `wait_for_tick()`（`__WFI` + tick pending 临界区），不再 `system_delay_ms`；PIT_CH0 ISR 末尾 `main_loop_on_pit_tick()` 唤醒。
- 调试观测：`s_main_tick_total ≈ 200/s`、`s_main_tick_overrun` 长期 0。

---

## 3. 视觉 ↔ 主控帧协议（P0-1）

```text
┌──────┬──────┬──────┬──────┬─────────────┬──────┐
│ 0xAA │ 0x55 │ TYPE │ LEN  │  PAYLOAD..  │ CRC8 │
└──────┴──────┴──────┴──────┴─────────────┴──────┘
TYPE: 0x01=MAP(192B ASCII, 12×16)  0x10=HEARTBEAT(1B seq)  0x02=BOX_CLASS(预留)
CRC8: poly=0x07, init=0x00, 覆盖 TYPE..PAYLOAD
```

- 主控侧 `app_link.c` 6 态状态机 + 50ms 字节超时；统计：`g_link_stats.{frames_ok, crc_err, len_err, byte_timeout, hb}`
- 视觉端 `F:\main.py` 提供 `pack_frame()` + `crc8()` + 100ms 心跳
- **链路超时回退（P0-2）**：>200ms 无心跳 → `STAGE_PAUSE_ON_LINK_LOSS`（保存现场 + 刹停）；恢复 100ms 后强制回 `STAGE_RECOGNIZE_MAP`（避免基于陈旧地图行动）；冷启动保护防止上电空转误触发。
- 对外查询：`Game_Link_Is_Alive()`（1=在线 / 0=离线）。

---

## 4. 游戏状态机

```
WAIT_START ──phase0: 复位起点 → phase1: 等"完全离开发车区"──► RECOGNIZE_MAP
RECOGNIZE_MAP ──收到完整 MAP 帧──► PLAN_PATH
PLAN_PATH ──Sokoban_Solve_Stage{1,2,3}──► EXECUTE_ACTION
EXECUTE_ACTION ──exec_push_waypoints 完成──► LEVEL_JUDGE
LEVEL_JUDGE ──下一关 / 完赛──► RECOGNIZE_MAP / DONE
DEADLOCK_RESET ──回发车区 + 静止≥3s──► WAIT_START
PAUSE_ON_LINK_LOSS ──链路恢复──► RECOGNIZE_MAP
DONE ──失败原因 Game_Get_Failure_Reason()──► (锁死)
```

**失败原因枚举** (`GameFailureReason_e`)：`NONE / OUT_OF_BOUNDS / DEADLOCK / TIMEOUT`

### 几何区域判定（P0-8）
全局坐标系 X∈[0, 3.20m], Y∈[0, 2.40m]。`chassis_ctrl.c` 提供：

| API | 含义 |
|---|---|
| `chassis_zone_tick()` | 主循环每 5ms 调用，更新 IIR 速度 / 静止累计 / OOB 滞回 latch |
| `chassis_zone_is_in_launch(LEFT/RIGHT/ANY)` | 圆 (R=0.175m) 与发车矩形相交 |
| `chassis_zone_is_fully_outside_launch()` | 圆与发车矩形完全分离 |
| `chassis_zone_is_out_of_bounds()` | 车体外接圆穿出黄边 >5cm（滞回） |
| `chassis_zone_is_static()` | 速度 <0.03 m/s 持续 3000ms |
| `chassis_zone_clear_oob()` | 复位 OOB latch（DEADLOCK 重置时调用） |

OOB 触发 → `chassis_ctrl_stop()` + 切 `STAGE_DONE` + `s_failure_reason = GAME_FAIL_OUT_OF_BOUNDS`。

---

## 5. 推箱子求解器

> 核心思路：[逐飞演示车模浅析](https://mp.weixin.qq.com/s/bzW6Kdvn7R8pHLWfXLsfSg) 中的"地图分解法"。

```text
原始地图 (N 个箱子 + N 个目标)
  ├─ 分解：对每个箱子生成子地图（其余箱子→墙，已完成→空地）
  ├─ 单箱 BFS：状态 = (玩家行列, 箱子行列)，状态空间 12×16×12×16=36864
  │   位图前沿 + 4-bit 前驱压缩存储 ≈ 31.5KB（旧版 108KB）
  ├─ 路点压缩：同方向连续移动合并为转弯点
  └─ 逐路点导航：chassis_ctrl_move_to_grid 逐点执行
```

### 三阶段 API

| 阶段 | 函数 | 策略 |
|---|---|---|
| 1 基础 | `Sokoban_Solve_Stage1()` | 贪心：最近箱→最近目标 |
| 2 分类 | `Sokoban_Solve_Stage2()` | OpenART 识别填 `g_box_to_target[]` 映射 |
| 3 策略 | `Sokoban_Find_Bomb_Wall()` + `Sokoban_Solve_Push_Bomb()` + `Sokoban_Apply_Bomb_Explosion()` | 不可达目标 → 收益评分选墙 → 推炸弹 → 爆破 3×3 → 重试 |

死局检测：`Sokoban_Is_Deadlock()`（角落卡死）。

### 桌面调试
将 `g_game_map` 转成 `# . $ @ ` 字符 + `sub_solutions[].actions` 转 UDLR 字符串，粘贴到 [SokoPlayer HTML5](https://sokoban.cn/sokoplayer/SokoPlayer_HTML5.php) 验证。

---

## 6. 引脚 & 关键参数

### 电机 / 编码器（2026-04-25 实车标定修正版，详见 `chassis_config.h`）
| 轮 | PWM | DIR | ENC A | ENC B | ENC 通道 |
|---|---|---|---|---|---|
| 左前 LF | C11 | C10 | C5  | C25 | QTIMER2_ENCODER2 (ENCODER_4) |
| 右前 RF | C6  | C7  | C3  | C4  | QTIMER2_ENCODER1 (ENCODER_3) |
| 左后 LB | D3  | D2  | C0  | C1  | QTIMER1_ENCODER1 (ENCODER_1) |
| 右后 RB | C8  | C9  | C2  | C24 | QTIMER1_ENCODER2 (ENCODER_2) |

### 通信 / 外设
| 用途 | 引脚 |
|---|---|
| OpenART UART1 | B12 (TX) / B13 (RX) |
| IPS200 SPI | 见 `zf_device_ips200` 默认引脚 |
| 按键 K1~K4 | C15 / C14 / C13 / C12 |
| IMU660RB | SPI（见 `chassis_config.h`） |

### 物理 / 控制（节选 `chassis_config.h`）
```c
#define CHASSIS_WHEEL_RADIUS_M          0.0315f
#define CHASSIS_HALF_WHEEL_BASE_M       0.100f
#define CHASSIS_HALF_TRACK_WIDTH_M      0.090f
#define CHASSIS_ENCODER_COUNTS_PER_REV  1024.0f
#define CHASSIS_WHEEL_PID_KP/KI/KD      120 / 8 / 1
#define CHASSIS_MAX_LINEAR_SPEED_MPS    0.35f
#define CHASSIS_LAUNCH_ZONE_W_M         0.30f
#define CHASSIS_LAUNCH_ZONE_H_M         0.30f
#define CHASSIS_BODY_RADIUS_M           0.175f
#define CHASSIS_OOB_HYSTERESIS_M        0.05f
#define CHASSIS_STATIC_HOLD_MS          3000
```
方向反了改 `CHASSIS_XX_DIR_SIGN` 为 `-1.0f`。

---

## 7. 编译与烧录

- **IAR EWARM**：打开 `project/iar/rt1064.eww`，`code` 分组已注册全部 `.c/.h`，直接 F7。
- **MDK5**：`project/mdk/rt1064.uvprojx` 的 `code` 分组为空，**未维护**。
- icf 栈/堆：`__size_cstack__=0x2000`（8KB，P0-4 扩容）、`__size_heap__=0x400`。

主入口：`project/user/src/main.c`；中断：`project/user/src/isr.c`。

---

## 8. 调试菜单（IPS200 + 4 按键）

一级菜单（按优先级）：`LIMIT → NAV → PID`
- K1 上一页 / K2 下一页 / K3 进入二级 / K4 长按保存 Flash

二级菜单（参数调节）：
- K1 上一项 / K2 下一项 / K3 + / K4 −
- K1 长按返回 / K4 长按保存 Flash

Flash 持久化：`sector=127, page=7`，启动时 magic/version/checksum 校验，失败则用默认值。

---

## 9. P0 任务进度（详见 `P0进度交接.md`）

| # | 任务 | 状态 |
|---|---|---|
| P0-1 | 帧协议加固（CRC8 + LEN + 心跳） | ✅ |
| P0-2 | 200ms 链路超时回退 | ✅ |
| P0-3 | `g_game_map` / `s_pose` seq-lock | ✅ |
| P0-4 | icf 栈扩容 + 大栈数组迁 static | ✅ |
| P0-5 | 主循环改 PIT tick + `__WFI` | ✅ |
| **P0-6** | **颜色 + 畸变实车标定走 SD csv** | ⏭️ 进行中 |
| P0-7 | 视觉端主循环 try/except 保护 | ⬜ |
| P0-8 | 发车区 / 越界几何判定 | ✅ |
| P0-9 | 软限位 `apply_soft_limit_guard` 全局启用 | ⬜ |
| P0-10 | 阶段一全程联调 | ⬜ |

### 已知 TODO（与状态机相关）
- `HAL_VISION_GET_BOX_CLASS_ID()` 占位返回 0，待对接 OpenART 分类帧
- `STAGE_OBSERVE_ALL` 遍历箱子 + 填 `g_box_to_target[]` 仍是框架代码
- `should_enter_next_level()` 占位返回 0

---

## 10. 协作铁律（继续遵守）

1. **绝对聚焦**：每次对话只做 1 个最高优先级任务，禁止跨任务/提前预判
2. **修改前沙盘推演**：先列文件清单 + 全局状态影响 + 防错边界，确认后再写代码
3. **工程级代码**：详尽中文注释、零阻塞、零无保护 `while(1)`、防错完整

---

## 11. 跨会话开场白模板

```
你是我的首席结对编程助手，请阅读工程根目录下的 P0进度交接.md 与 规则提炼.md，
继续按铁律 1/2/3 推进。当前进度已到 P0-6，请直接给出 P0-6 的沙盘推演并征求确认后实现。
工作区：e:\car\推箱子\visual-group + F:\
```
