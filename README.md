# 智能车视觉组 — 推箱子主控工程

> **平台**: NXP RT1064DVL6A (Cortex-M7 @ 600MHz)  
> **比赛**: 第 21 届全国大学生智能汽车竞赛 · AI 视觉组（推箱子赛题）  
> **底盘**: 4 麦克纳姆轮 + DRV8701E 双路驱动 + IMU660RB  
> **视觉**: OpenART Plus 副镜头（UART 115200，CRC8 协议帧）  
> **工程根目录**: `e:\car\visual-group`（主控 C 工程）；视觉脚本通常在盘符 `F:\`（OpenART SD / Python）  
> **最近更新**: 2026-05-07（README 已合并原《工程讲解与调试手册》，便于按问题定位源码）

---

## 调试快速索引（按现象 / 任务打开文件）

| 你想查什么 | 优先打开 |
|------------|----------|
| 串口帧解析、CRC、地图快照、seq-lock | `project/code/app_link.c`、`project/code/app_link.h` |
| UART 中断喂字节 | `project/user/src/isr.c`（`LPUART1_IRQHandler` → `app_link_isr_feed_byte`） |
| 链路 tick（5ms） | `app_link_tick()`，由 PIT_CH0 调用 |
| 游戏状态机、200ms 超时、PAUSE | `project/code/app_game_logic.c` |
| 底盘闭环、里程计、导航、`move_to_grid` | `project/code/chassis_ctrl.c` |
| 几何区域判定（发车区/越界/静止/软限位） | `project/code/chassis_zone.c`、`project/code/chassis_zone.h` |
| 位姿快照读 | `chassis_ctrl_get_pose()`（P0-3 seq-lock） |
| IMU、航向积分 | `project/code/chassis_imu.c` |
| 电机 PWM/DIR、编码器、麦轮、PID | `chassis_motor.c`、`chassis_encoder.c`、`chassis_mecanum.c`、`chassis_pid.c` |
| 引脚与物理/控制参数（改硬件只改这里） | `project/code/chassis_config.h` |
| IPS 菜单与 Flash 参数 | `project/code/chassis_menu.c` |
| 推箱子 BFS、炸弹策略 | `project/code/algo_sokoban_solver.c` |
| 主循环、`wait_for_tick`、`MAIN_*` 调试宏 | `project/user/src/main.c` |
| 链接脚本栈（HardFault 排查） | `project/iar/icf/MIMXRT1064xxxxx_flexspi_nor.icf` |
| 视觉端协议与网格采样 | `F:\main.py`（若路径不同以 SD 卡根目录 `main.py` 为准） |
| 官方引脚参考 | `project/RT1064智能车推荐引脚分配.txt` |
| 当前迭代任务与上下文 | `P0进度交接.md` |
| 比赛规则摘要 | `规则提炼.md` |

---

## 1. 工程用途与整体框图

四麦克纳姆轮 + RT1064 主控 + 双 OpenART Plus（第二路预留）的 **虚实结合 · 推箱子** 智能车：视觉端识别屏幕里 **16×12** 虚拟地图，主控跑状态机 + BFS 推箱子 + 麦轮闭环，在 **3.2×2.4 m** 物理场地完成虚拟关卡。

```text
              ┌─────────────────────────┐
              │   电脑端 3D 虚拟场景     │
              │ (Game Window 16x12 grid)│
              └──────────────┬──────────┘
                             ▼ (无线投屏 SpaceDesk/iVCam 等)
              ┌─────────────────────────┐
              │     车载手机/投屏屏幕    │
              └──────────────┬──────────┘
                             ▼ (镜头看屏)
   ┌──────────────────────┐         ┌────────────────────────┐
   │ OpenART Plus #1      │         │ OpenART Plus #2 (预留) │
   │  ─ F:\main.py        │         │  ─ 箱子分类 / MobileNet│
   │  ─ 网格采样 + k1     │         └────────┬───────────────┘
   │  ─ MAP + 心跳        │                  │ (P1)
   └─────────┬────────────┘
             ▼ UART 115200（SOF + CRC8）
   ┌──────────────────────────────────────────────────────────┐
   │                RT1064  (project/code)                      │
   │  app_link → app_game_logic → algo_sokoban_solver         │
   │       │            │                   │                   │
   │       └────────────┴──► chassis_ctrl（顶层底盘 API）      │
   │            imu / encoder / mecanum / pid / motor         │
   │  IPS200 + 4 键 → chassis_menu                              │
   └──────────────────────────────────────────────────────────┘
```

### 技术栈摘要

| 层 | 选型 |
|----|------|
| MCU | RT1064DVL6A @ 600 MHz Cortex-M7 |
| SDK | 逐飞 RT1064 开源库 `libraries/`（勿随意改 GPL 源码） |
| IDE | **IAR EWARM** 为主（`project/iar/rt1064.eww`）；MDK 工程 `code` 组未维护 |
| IMU | IMU660RB（SPI），上电静止约 1s 零偏 |
| 视觉 | OpenART Plus，MicroPython；与主控 UART 对接 |

---

## 2. 仓库目录结构

```text
visual-group/
├── README.md                 ← 本文件（含讲解 + 调试索引）
├── 工程讲解与调试手册.md       ← 已合并至 README，保留文件名便于旧链接
├── P0进度交接.md              ← 跨会话任务进度（必读）
├── 规则提炼.md                ← 比赛规则与策略
├── libraries/                 ← 逐飞 SDK（zf_common / zf_driver / zf_device / sdk）
└── project/
    ├── RT1064智能车推荐引脚分配.txt
    ├── code/                  ← 用户业务代码（IAR 工程 `code` 组）
    │   ├── chassis_config.h
    │   ├── chassis_pid.{c,h}
    │   ├── chassis_motor.{c,h}
    │   ├── chassis_encoder.{c,h}
    │   ├── chassis_imu.{c,h}
    │   ├── chassis_mecanum.{c,h}
    │   ├── chassis_ctrl.{c,h}
    │   ├── chassis_menu.{c,h}
    │   ├── algo_sokoban_solver.{c,h}
    │   ├── app_game_logic.{c,h}
    │   └── app_link.{c,h}
    ├── user/src/
    │   ├── main.c             ← 入口（含架构注释）
    │   └── isr.c              ← PIT / UART 等中断
    ├── iar/                   ← 主用工程与 icf
    └── mdk/                   ← 占位，需用时手动加源文件
```

**OpenART SD 卡根目录（典型 `F:\`，以实际盘符为准）**

```text
├── main.py                    ← 视觉主程序
├── cmm_load.py / cmm_cfg.csv ← 引脚映射（必备）
├── SD卡必备文件/              ← 备份上述必备文件
└── OpenART_Plus_1_map_code/   ← 第二路早期脚本等
```

---

## 3. `project/code/` 模块职责一览

| 模块 | 文件 | 职责 |
|------|------|------|
| 统一配置 | `chassis_config.h` | 引脚 / 物理尺寸 / PID / 限幅 / 几何（改硬件只改这里） |
| PID | `chassis_pid.c/.h` | 通用增量式 PID |
| 电机 | `chassis_motor.c/.h` | DRV8701E PWM + DIR |
| 编码器 | `chassis_encoder.c/.h` | 正交脉冲 → 轮速 |
| IMU | `chassis_imu.c/.h` | IMU660RB SPI，零偏标定 + 航向积分 |
| 麦轮运动学 | `chassis_mecanum.c/.h` | 正/逆运动学 + 等比例限速 |
| 顶层底盘 | `chassis_ctrl.c/.h` | 5/20ms 闭环 + 网格定点 + 航向保持 + `chassis_ctrl_move_to_grid()` |
| 几何判定 | `chassis_zone.c/.h` | 发车区 / 越界 / 静止判定 + 软限位预测保护（与 chassis_ctrl 解耦，仅消费 `chassis_ctrl_get_pose()` seq-lock 快照） |
| 调参菜单 | `chassis_menu.c/.h` | IPS + 4 按键 + Flash 持久化 |
| 推箱求解 | `algo_sokoban_solver.c/.h` | 地图分解 + 单箱位图 BFS + 炸弹策略 + 导航 BFS |
| 游戏状态机 | `app_game_logic.c/.h` | 主流程 + 链路守护 |
| 链路协议 | `app_link.c/.h` | CRC8 帧解析 + seq-lock 地图快照 + 心跳超时 |

---

## 4. 系统架构与并发要点

```text
                 ┌─────────────────────────────────────────────┐
   OpenART ──UART──► app_link.c (CRC8 解析 + seq-lock 写入)    │
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
   PIT_CH0 (5ms): IMU + 航向 + 链路 tick                       │
   PIT_CH1 (20ms): 编码器 → 里程计 → 导航 → 麦轮逆 → PID → PWM│
   PIT_CH2 (10ms): 菜单按键扫描                                 │
   └──────────────────────────────────────────────────────────┘
```

### 关键并发安全（P0-3）

- **`g_game_map`**: `app_link.c` 内 `s_map_authoritative` + `s_map_seq`；主循环每帧 `app_link_get_map_snapshot()` 取稳定快照。
- **`s_pose`**: `chassis_ctrl.c` 内 yaw（5ms ISR）/ x,y（20ms ISR）/ 主循环写均经 `pose_write_begin/end`；外部只读 `chassis_ctrl_get_pose()`。
- 重试放弃计数 `g_link_map_snapshot_retry_giveup` / `g_chassis_pose_snapshot_retry_giveup` 长期应保持 **0**。

### 主循环节拍（P0-5）

- 主循环使用 `wait_for_tick()`（`__WFI` + tick pending 临界区）；PIT_CH0 ISR 末尾 `main_loop_on_pit_tick()` 唤醒。
- 观测：`s_main_tick_total ≈ 200/s`、`s_main_tick_overrun` 长期为 0。

---

## 5. 视觉 ↔ 主控帧协议（P0-1）

```text
┌──────┬──────┬──────┬──────┬─────────────┬──────┐
│ 0xAA │ 0x55 │ TYPE │ LEN  │  PAYLOAD..  │ CRC8 │
└──────┴──────┴──────┴──────┴─────────────┴──────┘
TYPE: 0x01=MAP(推荐 194B：192B ASCII + car_x + car_y；兼容 192B 旧帧)
      0x10=HEARTBEAT(1B seq)  0x02=BOX_CLASS(预留)
CRC8: poly=0x07, init=0x00，覆盖 TYPE..PAYLOAD
```

- 主控：`app_link.c` 多态状态机 + 50ms 字节超时；统计 `g_link_stats.{frames_ok, crc_err, len_err, byte_timeout, hb}`。
- 示例心跳 seq=5：`AA 55 10 01 05 18`（可用 Python `pack_frame(0x10, b'\x05').hex()` 与 C 侧对照）。
- **链路超时（P0-2）**：>200ms 无心跳 → `STAGE_PAUSE_ON_LINK_LOSS`（保存现场 + 刹停）；恢复 100ms 后强制 `STAGE_RECOGNIZE_MAP`。冷启动保护避免误触发。
- 查询：`Game_Link_Is_Alive()`（1=在线 / 0=离线）。

### 数据流（ISR → 主循环）

```text
  OpenART UART(12), 115200              主控 LPUART1（引脚见 §8）
  ────────────────────────              ─────────────────────────────
  MAP/HB → pack_frame → 字节流    →    LPUART1_IRQHandler → app_link_isr_feed_byte
                                       PIT_CH0: app_link_tick(5)
                                       主循环: app_link_get_map_snapshot(g_game_map)
                                               chassis_ctrl_get_pose()
                                               Game_Logic_Task_Run()
```

---

## 6. 游戏状态机与几何区域

```
WAIT_START ──► RECOGNIZE_MAP ──► PLAN_PATH ──► EXECUTE_ACTION ──► LEVEL_JUDGE
      │              ▲                                              │
      └── DEADLOCK_RESET    PAUSE_ON_LINK_LOSS ──恢复──► RECOGNIZE_MAP
DONE（失败原因 Game_Get_Failure_Reason()）
```

**失败原因** (`GameFailureReason_e`)：`NONE / OUT_OF_BOUNDS / DEADLOCK / TIMEOUT`

### 几何区域（P0-8）

全局坐标 X∈[0, 3.20m], Y∈[0, 2.40m]。主循环需调用 `chassis_zone_tick()`。

| API | 含义 |
|-----|------|
| `chassis_zone_is_in_launch(LEFT/RIGHT/ANY)` | 车体圆与发车矩形相交 |
| `chassis_zone_is_fully_outside_launch()` | 完全离开发车区 |
| `chassis_zone_is_out_of_bounds()` | 外接圆穿出黄边 >5cm（滞回） |
| `chassis_zone_is_static()` | 速度 <0.03 m/s 持续 3000ms |
| `chassis_zone_clear_oob()` | 复位 OOB latch（如 DEADLOCK 重置） |

---

## 7. 推箱子求解器（摘要）

核心：**地图分解法** + 单箱位图 BFS + 路点压缩 + `chassis_ctrl_move_to_grid` 逐点执行。

| 阶段 | 函数 | 说明 |
|------|------|------|
| 1 | `Sokoban_Solve_Stage1()` | 贪心最近箱→目标 |
| 2 | `Sokoban_Solve_Stage2()` | 分类映射 `g_box_to_target[]` |
| 3 | 炸弹相关 API | 不可达时选墙、爆破 3×3、重试 |

死局：`Sokoban_Is_Deadlock()`。桌面验证：把 `g_game_map` 与 `actions` 转成字符地图 + UDLR，用 [SokoPlayer HTML5](https://sokoban.cn/sokoplayer/SokoPlayer_HTML5.php) 对照。

---

## 8. 硬件与引脚（主控 + 视觉）

### 8.1 电机 / 编码器（与 `chassis_config.h` 一致）

| 轮 | PWM | DIR | ENC A | ENC B | ENC 通道 |
|----|-----|-----|-------|-------|----------|
| LF | C11 | C10 | C5 | C25 | QTIMER2_ENCODER2 |
| RF | C6 | C7 | C3 | C4 | QTIMER2_ENCODER1 |
| LB | D3 | D2 | C0 | C1 | QTIMER1_ENCODER1 |
| RB | C8 | C9 | C2 | C24 | QTIMER1_ENCODER2 |

### 8.2 通信与外设

| 用途 | 说明 |
|------|------|
| OpenART 地图 UART | 常用 `UART4`：`MAIN_OPENART1_UART_TX/RX`（默认 C16/C17；若底板 D0/D1 则在 `chassis_config.h` 改宏） |
| Debug printf | 多与 `UART_1`、OpenART 链路共用同一物理口时需分清接线——看不到打印先确认接的是 MCU Debug TX，且串口工具 **关闭 HEX**、波特率 **115200** |
| IPS200 | SPI（见 `zf_device_ips200` / `chassis_menu`） |
| 按键 K1~K4 | C15 / C14 / C13 / C12 |
| IMU660RB | SPI，见 `chassis_config.h` |

> `C4~C15` 区域已被编码器与部分 PWM 大量占用；扩展硬件前对照 `project/RT1064智能车推荐引脚分配.txt`。

### 8.3 OpenART（`F:\main.py`）

| 项 | 说明 |
|----|------|
| 串口 | `UART(12)` 白色座，115200 |
| SD | 根目录必备 `cmm_load.py`、`cmm_cfg.csv` |
| 采样 | `params`：`ori_x/y`、`interval_x/y`、`k1` 畸变；`SYMBOL_MAP` 颜色（P0-6 实车标定） |

### 8.4 物理与控制节选（`chassis_config.h`）

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

方向反了改对应 `CHASSIS_*_DIR_SIGN` / `*_ENC_SIGN`。

---

## 9. 编译、烧录与可选 VS Code 工作流

- **IAR**：打开 `project/iar/rt1064.eww`，配置 `nor_sdram_zf_dtcm`，F7 编译；产物常见于 `project/iar/program/nor_sdram_zf_dtcm/Exe/RT106X.out`（名称以工程为准）。
- **命令行构建示例**（路径按本机 IAR 安装目录修改）：  
  `"…\IarBuild.exe" project\iar\program\rt1064.ewp -build nor_sdram_zf_dtcm`
- **VS Code**：若工作区存在 `.vscode/iar-vsc.json` / `launch.json`，可配合 **IAR Build / IAR Debug** 扩展一键编译与 F5 调试（CMSIS-DAP 或按 launch 配置改为 J-Link）。
- **MDK**：`project/mdk/rt1064.uvprojx` 的 `code` 分组可能为空，使用前需手动加入 `project/code/*.c`。
- **链接脚本**：`__size_cstack__=0x2000`（8KB，P0-4）、`__size_heap__=0x400`。**勿把栈改回 0x1000**，易 HardFault。
- **新增 `.c` 文件**：必须加入 `project/iar/program/rt1064.ewp` 的 `code` 组。
- **清理**：可用 `project/iar/IAR删除临时文件.bat`、`project/mdk/MDK删除临时文件.bat`。

主入口：`project/user/src/main.c`；中断：`project/user/src/isr.c`。

### OpenART SD 部署（简要）

1. SD 卡根目录放入 `cmm_load.py`、`cmm_cfg.csv`（见 `F:\SD卡必备文件` 备份）。  
2. 复制 `main.py` 到根目录，上电自动运行。  
3. IDE 中观察 `FPS`、`Recognized Map`；`MAP tx skip: bad payload len` 多出现时调采样参数与网格对齐。

---

## 10. 模块详解与调试 Checklist

### `chassis_config.h`

- 网格内外场、`CHASSIS_MAX_LINEAR_SPEED_MPS`、导航 `CHASSIS_POS_KP` / `CHASSIS_YAW_KP`、四轮 PWM/DIR/ENC、`CHASSIS_ODOM_SCALE_*`、`chassis_grid_*_to_m`。
- [ ] 改 `.h` 后必要时 **Rebuild All**（IAR 可能不全量重编依赖 `.c`）。

### `chassis_imu`

- 上电约 1s 静止零偏；`chassis_imu_task_5ms()`：死区 → LPF → 积分 `car_angle.yaw`；`volatile EulerAngle_t car_angle`。
- [ ] 上电车体静止；漂移则调 `CHASSIS_IMU_GYRO_DEADZONE_DPS` / `CHASSIS_IMU_BIAS_ADAPT_ALPHA`；yaw 反向改 `CHASSIS_IMU_YAW_SIGN`。

### 电机 / 编码器 / 麦轮 / PID

- PID 输入 m/s，输出占空比；麦轮 `K = 半轴距 + 半轮距`（默认 0.190 m）。
- [ ] T1 单轮：`MAIN_SINGLE_WHEEL_PID_DEBUG_MODE=1`；T2 编码器符号；T4 单轮 PID 先 P 后 I 后 D。

### `chassis_ctrl`

- `chassis_ctrl_task_5ms` / `_task_20ms`、`move_to_grid`、`get_pose`（seq-lock）、`set_pose`（视觉校正）。
- [ ] `g_chassis_pose_snapshot_retry_giveup == 0`；静止时 `wz_cmd` 收敛。

### `chassis_menu`

- `chassis_menu_task_10ms`（PIT_CH2）、`chassis_menu_render_100ms`（主循环）。

### `app_link`

- TYPE：`0x01` MAP、`0x10` HB、`0x02` BOX_CLASS 预留、`0x03` POSE_HINT 预留。
- [ ] `frames_ok` 递增；拔线 `frames_byte_timeout` 增；`crc_err/len_err` 长期为 0。

### `app_game_logic`

- [ ] 未接 OpenART 不误进 PAUSE（冷启动保护）；拔视觉 USB 约 200ms 刹停并 PAUSE；恢复后回 `RECOGNIZE_MAP`。

### `algo_sokoban_solver`

- 大数组已迁 `static`；注意 step 越界保护。

### `F:\main.py`

- `crc8` 与 C 侧一致；100ms 心跳；主循环外层异常包裹见 P0-7。
- [ ] FPS 目标 ≥30（理想 60）；payload 长度错误日志不应刷屏。

---

## 11. 关键参数速查表

| 参数 | 文件 | 默认 | 说明 |
|------|------|------|------|
| `MAIN_YAW_HOLD_TEST_MODE` | `main.c` | 视工程 | 1=航向调试，0=游戏逻辑 |
| `MAIN_SINGLE_WHEEL_PID_DEBUG_MODE` | `main.c` | 视工程 | 单轮 PID |
| `MAIN_PID_DEBUG_WHEEL_INDEX` | `main.c` | 如 RF | 调试轮索引 |
| `CHASSIS_MAX_LINEAR_SPEED_MPS` | `chassis_config.h` | 0.35 | 合成线速度上限 |
| `CHASSIS_POS_KP` / `CHASSIS_YAW_KP` | 同上 | 0.90 / 2.20 | 位置 / 航向环 |
| `CHASSIS_WHEEL_PID_*` | 同上 | 120/8/1 | 轮速 PID 默认 |
| `CHASSIS_ODOM_SCALE_X/Y` | 同上 | 标定值 | 里程计比例 |
| `LINK_LOSS_MS` / 恢复迟滞 | `app_game_logic.c` | 200 / 100 | 链路判断 |
| `params["k1"]` 等 | `main.py` | — | 畸变与网格步长 |
| `SYMBOL_MAP` | `main.py` | — | 颜色字典（P0-6） |
| `__size_cstack__` | `*.icf` | 0x2000 | 栈 |

---

## 12. 排错指南（常见现象 → 嫌疑点）

| 现象 | 排查 |
|------|------|
| 串口无输出 | 接线、`115200`、关 HEX、是否接到正确 TX |
| 中文乱码 | 工具编码 UTF-8；源文件损坏则从 Git 恢复 |
| `cnt` 远低于 200/s | 主循环阻塞；检查大屏刷新 `chassis_menu_render_100ms` |
| 上电 HardFault | icf 栈、大数组栈溢出；确认 `__size_cstack__=0x2000` |
| `frames_crc_err` | 波特率/接线/帧对齐；对照心跳示例帧 |
| `frames_byte_timeout` | OpenART 未发或断线 |
| 链路断不刹停 | `update_link_state`、`app_link_tick` 是否在 PIT_CH0 |
| 恢复后未回识图 | `stage_pause_on_link_loss_handler` 是否强制 `RECOGNIZE_MAP` |
| Yaw 漂移 | 上电静止；死区 / 零偏自适应 |
| 单轮狂转 / 编码器符号错 | `*_DIR_SIGN`、`*_ENC_SIGN` |
| OpenART `bad payload len` | `ori_*`、`interval_*`、`k1`、采样越界 |
| OpenART 不运行 | SD 缺 `cmm_load.py` / `cmm_cfg.csv` |
| IPS 黑屏 / 按键无效 | SPI 初始化、`key_init`、`chassis_menu_task_10ms` 是否在 PIT |

### 常用观测输出位置

| 内容 | 位置 |
|------|------|
| 单轮 PID | `chassis_ctrl.c` → `chassis_pid_debug_task_5ms` |
| 航向闭环 | `chassis_ctrl_attitude_debug_task_5ms` |
| 链路统计 | 主循环按需 `printf` `g_link_stats` |
| 视觉 | `main.py` 控制台 FPS / Map |

---

## 13. 联调测试顺序（T0–T13，原则摘要）

自底向上、单步隔离、**车轮悬空优先**。

| 顺序 | 代号 | 焦点 |
|------|------|------|
| 1 | T0 | 时基 + 串口打印正常 |
| 2 | T1 | PWM / DIR 极性 |
| 3 | T2 | 编码器方向与计数 |
| 4 | T3 | IMU 零偏与 yaw |
| 5 | T4 | 单轮 PID |
| 6 | T5 | 麦轮开环合成 |
| 7 | T6 | 姿态闭环 |
| 8 | T7 | 里程计标定 |
| 9 | T8 | 视觉单机自检 |
| 10 | T9 | 链路帧与心跳 |
| 11 | T10 | 全局坐标 / 网格映射 |
| 12 | T11 | 点到点导航 |
| 13 | T12 | BFS + 全程 |

每项建议包含：目标 → 软硬件准备 → 操作步骤 → 量化期望 → 故障时代码定位。若仓库中曾有的独立《测试方案》文档已移除，可按上表在 `P0进度交接.md` 或 Issue 中延续记录测试结果。

---

## 14. 调试菜单（IPS200 + 4 按键）

一级：`LIMIT → NAV → PID` — K1 上页 / K2 下页 / K3 进二级 / K4 长按存 Flash。  
二级：K1 上项 / K2 下项 / K3 + / K4 −；K1 长按返回；K4 长按存 Flash。  
Flash：`sector=127, page=7`，magic/version/checksum。

---

## 15. P0 任务进度（详见 `P0进度交接.md`）

| # | 任务 | 状态 |
|---|------|------|
| P0-1 | 帧协议加固（CRC8 + LEN + 心跳） | ✅ |
| P0-2 | 200ms 链路超时回退 | ✅ |
| P0-3 | `g_game_map` / `s_pose` seq-lock | ✅ |
| P0-4 | icf 栈扩容 + 大栈数组迁 static | ✅ |
| P0-5 | 主循环 PIT tick + `__WFI` | ✅ |
| P0-6 | 颜色 + 畸变实车标定（如 SD csv） | ⏭️ 进行中 |
| P0-7 | 视觉主循环 try/except 保护 | ⬜ |
| P0-8 | 发车区 / 越界几何判定 | ✅ |
| P0-9 | 软限位全局启用 | ⬜ |
| P0-10 | 阶段一全程联调 | ⬜ |

### 已知 TODO（与状态机相关）

- `HAL_VISION_GET_BOX_CLASS_ID()` 占位返回 0，待对接分类帧  
- `STAGE_OBSERVE_ALL` 填 `g_box_to_target[]` 仍为框架  
- `should_enter_next_level()` 占位返回 0  

---

## 16. 协作约定（铁律 + 流程）

1. **绝对聚焦**：单次迭代优先一个最高优先级任务。  
2. **修改前沙盘推演**：文件清单、全局状态影响、边界条件。  
3. **工程级代码**：中文注释、避免 ISR 内阻塞、避免无超时 `while(1)`。  
4. **命名**：底盘 `CHASSIS_`；链路 `APP_LINK_` / `PROTO_`；状态 `STAGE_`。  
5. **编码**：UTF-8（无 BOM）；改引脚 / 协议 / P0 任务时同步更新本文档相关小节。  
6. **勿随意修改** `libraries/` 内逐飞 SDK；主循环节拍勿再依赖 `system_delay_ms` 凑周期。

---

## 17. 跨会话开场白模板（可复制）

```
你是我的首席结对编程助手，请阅读工程根目录下的 P0进度交接.md 与 规则提炼.md，
继续按 README §16 铁律推进。当前进度已到 P0-6（以交接文档为准），先给出沙盘推演再改代码。
工作区：e:\car\visual-group（及视觉盘 F:\，若适用）
```

---

> **说明**：原独立文档《工程讲解与调试手册》的主要内容已合并至本 README；调试时请优先使用文首 **「调试快速索引」** 定位源文件。
