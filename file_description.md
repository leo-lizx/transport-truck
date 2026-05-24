# code 文件夹文件说明

> 本文档描述 `project/code/` 目录下每个文件的职责、内部关键接口，以及模块之间的调用关系。

---

## 目录

1. [架构总览](#1-架构总览)
2. [文件一览表](#2-文件一览表)
3. [分层说明](#3-分层说明)
   - [3.1 运行时基础层](#31-运行时基础层)
   - [3.2 算法层](#32-算法层)
   - [3.3 底盘驱动层](#33-底盘驱动层)
   - [3.4 底盘控制层（唯一对外入口）](#34-底盘控制层唯一对外入口)
   - [3.5 应用层](#35-应用层)
4. [调用关系图](#4-调用关系图)

---

## 1. 架构总览

```
┌─────────────────────────────────────────────────────────┐
│                      main.c（入口）                      │
│  配置运行模式 → 底盘初始化 → 主循环 5ms/20ms tick 分发    │
└───────────────────────┬─────────────────────────────────┘
                        │
        ┌───────────────┼──────────────────┐
        ▼               ▼                  ▼
  app_main_modes   app_main_runtime    chassis_ctrl ─── chassis_imu
  （模式分发）     （PIT/UART桥接）    （底盘唯一    ─── chassis_encoder
        │                                  入口）    ─── chassis_motor
        ├── app_game_logic （完整赛场流程）           ─── chassis_pid
        │        ├── app_link         （串口解析）    ─── chassis_mecanum
        │        ├── app_recognize    （识别巡逻）    ─── chassis_zone
        │        └── app_vision_fusion（位姿融合）    ─── chassis_config
        │
        └── [静态地图调试模式]
                 ├── app_static_map_drive_helper
                 └── algo_sokoban_solver（推箱算法）
```

**视觉端通信**：OpenART Plus 通过 UART4 发送 `0xAA 0x55` 二进制帧 → `app_link.c` ISR 逐字节解析 → 主循环通过 `app_link_get_map_snapshot()` 读取一致地图副本。

---

## 2. 文件一览表

| 文件（.c / .h）                  | 所属层    | 一句话职责                                   |
|----------------------------------|-----------|----------------------------------------------|
| `algo_sokoban_solver`            | 算法层    | 推箱子 BFS 求解算法（完全独立于硬件）        |
| `app_game_logic`                 | 应用层    | 游戏主状态机（比赛全流程顶层调度）           |
| `app_link`                       | 应用层    | OpenART 串口帧协议解析（MAP / 心跳 / 识别）  |
| `app_main_config`                | 基础层    | 系统级常量（纯头文件，无 .c）                |
| `app_main_modes`                 | 应用层    | 运行模式分发（调试模式 / 比赛模式切换）      |
| `app_main_runtime`               | 基础层    | PIT tick / UART4 探针 ISR 桥接               |
| `app_recognize`                  | 应用层    | 识别巡逻子状态机（Stage2 / Stage3 路线规划） |
| `app_static_map_drive_helper`    | 应用层    | 静态地图字符转换与辅助工具函数               |
| `app_vision_fusion`              | 应用层    | 视觉位姿 Snap 融合（到站校正 + 一致性监控）  |
| `chassis_config`                 | 驱动层    | 底盘硬件引脚 / 调参结构体 / Flash 持久化     |
| `chassis_ctrl`                   | 控制层    | 底盘顶层控制（整合编码器 / IMU / PID / 导航）|
| `chassis_encoder`                | 驱动层    | 正交编码器测速（A/B 双通道）                 |
| `chassis_imu`                    | 驱动层    | IMU 航向角积分（yaw，5ms 周期）              |
| `chassis_mecanum`                | 驱动层    | 麦克纳姆轮正逆运动学                         |
| `chassis_menu`                   | 驱动层    | IPS 屏幕菜单 / 按键调参                      |
| `chassis_motor`                  | 驱动层    | 电机 PWM 驱动（DRV8701E，PWM + 方向引脚）    |
| `chassis_pid`                    | 驱动层    | 增量式 PID 控制器（四轮独立轮速闭环）        |
| `chassis_zone`                   | 驱动层    | 发车区 / 越界 / 静止判定                     |

---

## 3. 分层说明

### 3.1 运行时基础层

#### `app_main_config.h`（纯头文件，无 .c）

系统级常量定义，不含任何可执行代码，所有模块都可安全包含。

| 常量 / 宏 | 含义 |
|---|---|
| `MAIN_OPENART1_UART` | OpenART UART 通道号（UART4） |
| `MAIN_POS_NAV_START_X/Y_GRID` | 发车点网格坐标（1.0, 5.5），Y 含半格偏移 |
| `MAIN_POS_GRID_TO_M_X/Y(g)` | 网格坐标 → 米制坐标转换宏 |
| `MAIN_POINT_NAV_WARMUP_TICKS` | 静态地图模式启动预热 tick 数（200 × 5ms） |

**被谁包含**：`app_main_modes.h`、`chassis_config.h`，进而扩散到所有模块。

---

#### `app_main_runtime.c / .h`

主循环节拍管理与 ISR 桥接，职责极轻，不做任何业务逻辑。

| 函数 | 调用者 | 说明 |
|---|---|---|
| `main_loop_on_pit_tick()` | **PIT 中断** | 只递增 `volatile s_main_tick_pending`，不可做其他事 |
| `App_MainRuntime_WaitForTick()` | `main.c` 主循环 | 阻塞等待至少 1 个 5ms tick，返回本次消费的 tick 数 |
| `main_uart4_tap_byte(b)` | **UART4 中断** | 探针开启时把 1 字节写入 1 KB 诊断环形缓冲 |
| `App_MainRuntime_DrainUart4Tap5ms()` | `app_main_modes.c` | 主循环中排空环形缓冲并打印，**禁止在中断中调用** |
| `App_MainRuntime_EnableUart4Tap()` | `app_main_modes.c` | 开启探针，清空缓冲区（关中断保护） |

---

### 3.2 算法层

#### `algo_sokoban_solver.c / .h`

推箱子核心求解算法，**完全独立于硬件**，可在 PC 验证器中复用。

**核心思路**：将 N 个箱子拆解为 N 个独立单箱子问题，每个子问题用 BFS（状态 = 玩家坐标 + 箱子坐标）求解，子地图中未完成箱子当墙处理。

| 函数 | 输入 | 输出 | 说明 |
|---|---|---|---|
| `Sokoban_Solve_Stage1(map, player, &result)` | 地图 + 起点 | 求解结果（含每段动作序列） | 贪心分配：最近箱子优先；返回 1=已解 |
| `Sokoban_Solve_Stage2(map, player, box_to_target, &result)` | 地图 + 起点 + 指定映射 | 同上 | 按识别阶段确定的 box→target 映射强制分配 |
| `Sokoban_Actions_To_Waypoints(actions, count, start, &path)` | 动作序列 + 起点 | 转弯路点数组 | 同向连续动作压缩为一个路点，减少底盘下发频次 |
| `Sokoban_Find_Bomb_Wall(map, bomb_pos, &wall, &actions)` | 地图 + 炸弹坐标 | 墙体位置 + 动作序列 | Stage3 炸弹推送辅助 |

**被谁调用**：
- `app_game_logic.c` → Stage1/2 求解 + 转路点
- `app_main_modes.c` → 静态地图调试模式直接调用 Stage1 + 转路点
- `app_recognize.c` → 辅助计算识别巡逻顺序（BFS 最短路）

---

### 3.3 底盘驱动层

#### `chassis_config.c / .h`

底盘唯一的"配置数据库"，所有可调参数集中在此，其他模块只读不直接存储调参值。

- **硬件结构宏**：引脚编号、编码器通道、PWM 通道、网格参数（16×12）、物理尺寸（3.2×2.4 m）。
- **调参结构体** `chassis_tune_params_t`：按子结构组织 —— `wheel_pid`（四轮速度 PID）、`position`（位置环参数）、`yaw`（航向角环）、`limit`（速度 / 加速度上限）、`odom`（里程计比例）等。
- **Flash 持久化**：`chassis_config_save_to_flash()` / `chassis_config_load_from_flash()`，双槽滚动，含 magic/version/CRC32，上电自动回退到内置默认值。
- **活动参数读取**：`chassis_config_get()` 返回当前活动参数只读指针。

**写者**：`chassis_menu.c`（用户按键修改并保存）或上电时 Flash 自动加载。  
**被谁包含**：几乎所有 `chassis_*` 和 `app_*` 模块。

---

#### `chassis_encoder.c / .h`

正交编码器测速，封装 MCU 定时器硬件计数通道。

- `chassis_encoder_init(enc)` → 配置 A/B 两路引脚和计数方向。
- `chassis_encoder_update(enc, dt_s)` → **20ms 周期调用**，读取脉冲增量换算 m/s。
- `chassis_encoder_get_speed(enc)` → 返回最新轮速（m/s）。

**被谁调用**：`chassis_ctrl.c` → 20ms 任务为四轮依次调用。

---

#### `chassis_imu.c / .h`

IMU 航向角积分（只使用 yaw 轴，roll/pitch 保留字段不参与控制）。

- `chassis_imu_init()` → 硬件初始化 + 静态零偏标定，**调用时车体必须静止**。
- `chassis_imu_update_5ms()` → **5ms 周期调用**，yaw 积分 + 零偏在线辨识 + 卡尔曼滤波。
- `chassis_imu_get/set_yaw_deg()` → 外部读写当前航向角。
- `chassis_imu_kf_angle_meas_update(yaw_obs, R)` → 外部（如里程计 yaw 估算）输入弱约束观测量。

**被谁调用**：`chassis_ctrl.c` → 5ms 任务更新；20ms 任务读 yaw 用于导航控制。

---

#### `chassis_mecanum.c / .h`

麦克纳姆轮运动学，纯数学计算，无硬件依赖。

- `chassis_mecanum_forward(vx, vy, wz, out[4])` → **正运动学**：车体速度 → 四轮目标速度，用于下发控制指令。
- `chassis_mecanum_inverse(wheel[4], &vx, &vy)` → **逆运动学**：四轮实际速度 → 车体速度，用于里程计估算。
- `chassis_mecanum_clamp_wheels(wheel[4], max)` → 等比例限速，保持运动方向不变。

**被谁调用**：`chassis_ctrl.c` → 20ms 任务中正逆运动学分别调用。

---

#### `chassis_motor.c / .h`

DRV8701E 电机驱动板封装（PWM 占空比 + 方向引脚）。

- `chassis_motor_init(motor)` → 配置 PWM 频率和方向 GPIO，初始 PWM=0。
- `chassis_motor_set_pwm(motor, pwm_signed)` → 正值正转，负值反转，超限自动钳位。
- `chassis_motor_stop(motor)` → 紧急停止，PWM 置 0。

**被谁调用**：`chassis_ctrl.c` → 20ms 任务 PID 输出后为四轮调用。

---

#### `chassis_pid.c / .h`

增量式 PID 控制器，专用于四轮独立轮速闭环。

增量公式：`Δu(k) = Kp·[e(k)-e(k-1)] + Ki·e(k) + Kd·[e(k)-2e(k-1)+e(k-2)]`

- `chassis_pid_init(pid, kp, ki, kd, max)` → 设置增益和输出限幅。
- `chassis_pid_step(pid, target, feedback)` → 单步计算，返回当前累计输出。
- `chassis_pid_reset(pid)` → 模式切换 / 急停时清零状态。

**被谁调用**：`chassis_ctrl.c` → 20ms 任务为四轮各调用一次。

---

#### `chassis_zone.c / .h`

底盘几何区域判定，从 `chassis_ctrl.c` 拆分出来以保持关注点分离。

| 函数 | 调用节拍 | 说明 |
|---|---|---|
| `chassis_zone_tick()` | 5ms | 刷新位姿快照、速度估算、越界滞回标志 |
| `chassis_zone_is_in_launch(zone)` | 任意主循环 | 车体是否在指定发车区内 |
| `chassis_zone_is_fully_outside_launch(zone)` | 任意主循环 | 车体外接圆是否完全离开发车区 |
| `chassis_zone_is_out_of_bounds()` | 任意主循环 | 是否越过最外圈围墙（滞回，需手动清除） |
| `chassis_zone_apply_soft_limit_guard(cmd)` | 20ms（`chassis_ctrl` 内） | 地图边界软限位，修改速度指令防撞墙 |

**被谁调用**：`chassis_ctrl.c`（软限位）、`app_game_logic.c`（越界检测）。

---

#### `chassis_menu.c / .h`

IPS 屏幕菜单与按键实时调参，依赖 `chassis_config` 读写活动参数。

- `chassis_menu_init()` → **启动时调用一次**，加载当前调参并初始化内部状态。
- `chassis_menu_task_10ms()` → **10ms 周期**，扫描按键、切换菜单项、修改参数、触发 Flash 保存。
- `chassis_menu_render_100ms()` → **100ms 周期**，把当前活动参数渲染到 IPS 屏幕。

**被谁调用**：`main.c` → 主循环按各自节拍调用三个接口。

---

### 3.4 底盘控制层（唯一对外入口）

#### `chassis_ctrl.c / .h`

整合所有底盘子模块，是上层业务代码与底盘硬件之间的**唯一接口**。上层模块（`app_game_logic`、`app_main_modes` 等）不需要直接调用 motor / encoder / imu 等模块。

**初始化**：`chassis_ctrl_init()` → 依次初始化 IMU、编码器、电机、PID，初始电机停止。

**5ms 高频任务** `chassis_ctrl_task_5ms()`：
1. 调用 `chassis_imu_update_5ms()` 更新 yaw 积分。

**20ms 中频任务** `chassis_ctrl_task_20ms()`：
1. `chassis_encoder_update × 4` → 四轮实际速度
2. `chassis_mecanum_inverse` → 车体速度（逆运动学）
3. 里程计积分 → 更新全局位姿 (x, y)
4. 导航 P/PI 控制器 → 计算目标车体速度（轴向独立移动 + CTE 横向保持）
5. 速度缓加速滤波 → `chassis_mecanum_forward`（正运动学）→ `chassis_pid_step × 4` → `chassis_motor_set_pwm × 4`

**主要外部 API**：

| API | 说明 |
|---|---|
| `chassis_ctrl_move_to_grid(x, y)` | 下发网格坐标目标点，非阻塞 |
| `chassis_ctrl_is_arrived()` | 查询是否到点（1=已到） |
| `chassis_ctrl_stop()` | 立即停止所有电机 |
| `chassis_ctrl_get_pose()` | 读取全局位姿（seq-lock 线程安全） |
| `chassis_ctrl_set_pose(x_m, y_m, yaw_deg)` | 外部写入位姿（视觉位姿校正用） |
| `chassis_ctrl_rotate_to_deg(deg)` | 原地旋转到目标航向角 |

**被谁调用**：`main.c`（初始化和 tick）、`app_game_logic.c`（通过 HAL 宏）、`app_main_modes.c`（调试模式直调）、`app_vision_fusion.c`（`set_pose` 重定位）、`app_recognize.c`（移动和旋转）。

---

### 3.5 应用层

#### `app_link.c / .h`

OpenART Plus（视觉板）→ RT1064（主控）串口帧协议解析层。

**帧格式**：`[0xAA][0x55][TYPE][LEN][PAYLOAD...][CRC8]`，CRC8 多项式 0x07，覆盖 TYPE 到 PAYLOAD。

| 帧类型 | 值 | 载荷大小 | 说明 |
|---|---|---|---|
| MAP | 0x01 | 192B（或 194B 含车坐标） | 视觉 → 主控：全局地图 ASCII 字符 |
| BOX_CLASS | 0x02 | 3B (obj_kind + class_id + seq) | 视觉 → 主控：物体分类识别结果 |
| HEARTBEAT | 0x10 | 1B (seq) | 视觉 → 主控：链路保活心跳 |

**线程安全设计**：UART4 ISR 调用 `App_Link_FeedByte()` 写内部 seq-lock 缓冲区；主循环通过 `app_link_get_map_snapshot()` 原子拷贝，保证读到完整一致的地图。

**被谁调用**：
- UART4 中断 → `App_Link_FeedByte()`
- `app_game_logic.c` → `app_link_get_map_snapshot()` 每 5ms 刷新地图
- `app_recognize.c` → `app_link_get_box_class_snapshot()` 获取识别结果
- `chassis_zone.c` → `app_link_get_map_snapshot()` 用于软限位地图查询

---

#### `app_game_logic.c / .h`

游戏主状态机，**最顶层业务逻辑**，负责全赛场自动流程编排。

**状态机流程**：

```
WAIT_START         — 等待车体完全离开发车区（赛规要求）
    │
    ▼
RECOGNIZE_MAP      — 识别阶段（调用 App_Recognize_Tick）
    │               识别各箱子类别，建立 box→target 映射
    ▼
PLAN_PATH          — 调用 Sokoban_Solve_Stage1/2 求解推箱路线
    │               调用 Sokoban_Actions_To_Waypoints 生成航点
    ▼
EXECUTE_ACTION     — 逐点下发航点给 chassis_ctrl_move_to_grid
    │               等待 chassis_ctrl_is_arrived 后切换下一点
    ▼
LEVEL_JUDGE        — 当前关完成，判断是否进入下一关（共 3 关）
    │
    ├── DEADLOCK_RESET  — 死局：回发车区静止 3s 后重置
    ├── PAUSE_ON_LINK_LOSS — 链路超时刹停，恢复后续跑
    └── DONE            — 全流程完成 / 越界判罚终止
```

**外部接口**：

| 接口 | 说明 |
|---|---|
| `Game_Logic_Task_Run()` | 每 5ms 调用，驱动整个状态机 |
| `Game_Link_Is_Alive()` | 视觉链路是否在线 |
| `Game_Get_Failure_Reason()` | 查询失败原因（越界等） |
| `g_game_map[MAP_ROWS][MAP_COLS]` | 当前地图快照（主循环唯一写者） |

**被谁调用**：`app_main_modes.c` → `APP_RUN_MODE_GAME` 模式每 5ms 调用 `Game_Logic_Task_Run()`。

---

#### `app_recognize.c / .h`

识别巡逻子状态机，对应 `STAGE_RECOGNIZE_MAP` 阶段，实现 Stage2/Stage3 所需的物体分类。

**执行步骤**（每个待识别物体循环一次）：
1. 提取地图中所有 BOX / TARGET 坐标，贪心 BFS 决定访问顺序。
2. 移动到物体侧面可立足观察点（`chassis_ctrl_move_to_grid`）。
3. 原地旋转车头朝向物体（`chassis_ctrl_rotate_to_deg`）。
4. 多数票采样 BOX_CLASS 帧（`app_link_get_box_class_snapshot`），稳定后记录结果。
5. 全部完成后写入 `g_box_to_target[]` 映射，供 Stage2 求解使用。

**外部接口**：`App_Recognize_Tick()` — 每 5ms 由 `app_game_logic.c` 调用，返回 `APP_RECOG_RUNNING / DONE_OK / DONE_NO_NEED / FAIL`。

---

#### `app_vision_fusion.c / .h`

视觉位姿与里程计融合，防止长途行驶后里程计漂移累积。

**三类能力**：

| 能力 | 函数 | 典型场景 |
|---|---|---|
| Snap 表决（主要） | `app_vision_fusion_snap_request()` | 到站静止后多帧表决，通过则调用 `chassis_ctrl_set_pose()` 把位姿 Snap 到格中心 |
| 一致性监控 | `app_vision_fusion_consistency_tick()` | 运动中比较里程计格 vs 视觉格的曼哈顿距离，超阈值持续够久则硬重定位 |
| 连续软融合 | `app_vision_fusion_task()` | 默认编译为空（`CHASSIS_VISION_FUSION_ENABLE==0`），推荐保持关闭 |

**被谁调用**：`app_main_modes.c` / `app_game_logic.c` → 到达航点后调用 `snap_request()`。

---

#### `app_static_map_drive_helper.c / .h`

静态地图驱动的工具函数集，不含状态机，只提供无副作用的转换和辅助求解。

| 函数 | 说明 |
|---|---|
| `App_StaticMapDrive_CharToMap(ch)` | ASCII 字符 → `MAP_*` 枚举（`#`→墙, `-`→空地, `$`→箱子等） |
| `App_StaticMapDrive_MapToChar(v)` | `MAP_*` 枚举 → ASCII（用于日志打印确认地图内容） |
| `App_StaticMapDrive_LoadCharMap(src, dst)` | 整张手写 ASCII 地图批量加载为枚举数组 |
| `App_StaticMapDrive_SolveFromLaunch(...)` | 枚举半格发车点入口候选、择优求解（备用，当前 SMD 模式已改为直接调用 Stage1） |

**被谁调用**：`app_main_modes.c`（`LoadCharMap` 加载手写地图；`MapToChar` 打印日志）。

---

#### `app_main_modes.c / .h`

运行模式分发层，是 `main.c` 与各功能业务之间的**调度桥接**。

**运行模式**（`app_main_run_mode_e`）：

| 枚举值 | 用途 |
|---|---|
| `APP_RUN_MODE_GAME` | 完整比赛流程（调用 `Game_Logic_Task_Run`） |
| `APP_RUN_MODE_YAW_HOLD` | 航向保持调试（验证 IMU yaw 环稳定性） |
| `APP_RUN_MODE_SINGLE_WHEEL` | 单轮速度 PID 调试（标定编码器和起步补偿） |
| `APP_RUN_MODE_POINT_NAV` | 点到点导航调试（验证定位精度和到点判定） |
| `APP_RUN_MODE_SOKO_SELFTEST` | 推箱求解器离线自测（只打印结果，不运动） |
| `APP_RUN_MODE_OPENART1_TEST` | OpenART1 串口链路测试 |
| `APP_RUN_MODE_STATIC_MAP_DRIVE` | 静态地图全自动推箱执行（当前默认模式） |

**静态地图模式（SMD）内部状态机**：

```
WAIT_MAP    → 等待地图来源（手写地图 or OpenART MAP 帧）
    │
    ▼
WARMUP      → 预热 200 tick（1s）让底盘稳定
    │
    ▼
PUSH_BOXES  → 逐点下发 s_smd_waypoints 中的路点
              s_smd_wp_idx 递增直到 == waypoints.count
    │
    ▼
RETURN_HOME → 下发返回原点指令
    │
    ▼
DONE
```

**主要外部接口**：

| 接口 | 说明 |
|---|---|
| `App_MainModes_Config(options)` | 底盘初始化前配置运行模式和调试参数 |
| `App_MainModes_InitCommunication()` | 初始化当前模式需要的 UART（底盘初始化前调用） |
| `App_MainModes_AfterChassisInit()` | 底盘初始化完成后启动当前模式状态 |
| `App_MainModes_Task5ms()` | 主循环 5ms tick 分发到当前模式 |
| `App_MainModes_ShouldRenderMenu()` | 返回 1 表示允许底盘菜单刷新屏幕 |

**被谁调用**：`main.c` → 启动序列和主循环。

---

## 4. 调用关系图

```
main.c
│
├── App_MainModes_Config / InitCommunication / AfterChassisInit  （启动序列）
│
├── App_MainRuntime_WaitForTick()            （主循环 5ms 节拍）
│
├── App_MainModes_Task5ms()                  （5ms 业务分发）
│         │
│         ├── [APP_RUN_MODE_GAME]
│         │     └── Game_Logic_Task_Run()
│         │              ├── app_link_get_map_snapshot()     写入 g_game_map
│         │              ├── chassis_zone_tick()              越界 / 区域判定
│         │              ├── App_Recognize_Tick()            识别巡逻子状态机
│         │              │       ├── chassis_ctrl_move_to_grid / is_arrived
│         │              │       └── app_link_get_box_class_snapshot()
│         │              ├── Sokoban_Solve_Stage1 / Stage2   路线求解
│         │              ├── Sokoban_Actions_To_Waypoints    生成路点
│         │              └── chassis_ctrl_move_to_grid       逐点执行
│         │
│         └── [APP_RUN_MODE_STATIC_MAP_DRIVE]
│               ├── App_StaticMapDrive_LoadCharMap()         加载手写地图
│               ├── App_StaticMapDrive_MapToChar()           日志打印
│               ├── Sokoban_Solve_Stage1()                   直接求解
│               ├── Sokoban_Actions_To_Waypoints()           生成路点
│               └── chassis_ctrl_move_to_grid()              逐点下发
│
├── chassis_ctrl_task_5ms()                  （5ms 高频）
│         └── chassis_imu_update_5ms()
│
└── chassis_ctrl_task_20ms()                 （20ms 控制周期）
          ├── chassis_encoder_update × 4     读四轮速度
          ├── chassis_mecanum_inverse()      逆运动学 → 车体速度
          ├── 里程计积分 → 更新位姿 (x, y)
          ├── 导航控制器 → 目标车体速度（轴向独立 + CTE 保持）
          ├── chassis_mecanum_forward()      正运动学 → 四轮目标速度
          ├── chassis_pid_step × 4           PID 闭环
          └── chassis_motor_set_pwm × 4     输出到电机
```

> **注**：`app_vision_fusion_snap_request()` 由 `app_game_logic.c` 或 `app_main_modes.c` 在到点后调用，内部异步执行表决，通过后调用 `chassis_ctrl_set_pose()` 完成位姿校正，图中未展开。
