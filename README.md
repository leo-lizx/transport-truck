# 智能车视觉组 — 底盘控制子系统

> **平台**: NXP RT1064DVL6A (i.MX RT Cortex-M7)  
> **比赛**: 第 21 届全国大学生智能汽车竞赛 · AI 视觉组  
> **底盘**: 4 麦克纳姆轮 + DRV8701E 双路驱动 + IMU660RB 6 轴 IMU

---

## 目录结构

```text
project/
├── code/                          ← 用户代码（所有 .c/.h 平铺存放）
│   ├── chassis_config.h           ← ★ 引脚 & 参数统一配置（改硬件只改这里）
│   ├── chassis_pid.c/.h           ← 增量式 PID 控制器
│   ├── chassis_motor.c/.h         ← 电机驱动（PWM + 方向）
│   ├── chassis_encoder.c/.h       ← 编码器测速
│   ├── chassis_imu.c/.h           ← IMU660RB 航向角积分
│   ├── chassis_mecanum.c/.h       ← 麦轮正/逆运动学
│   ├── chassis_ctrl.c/.h          ← ★ 底盘顶层控制（对外唯一入口）
│   ├── chassis_pose_ctrl_call_example.c/.h  ← 调用层封装（供 main/isr 使用）
│   ├── chassis_pose_ctrl.c/.h     ← 旧文件兼容层（已清空/宏映射）
│   ├── chassis_menu.c/.h          ← 2寸 IPS 参数菜单（按键在线调参）
│   ├── algo_bfs_scout.c/.h        ← BFS 纯寻路 + 最近箱子观察点搜索
│   ├── algo_sokoban_solver.c/.h   ← ★ 推箱子求解器（地图分解法 + 单箱 BFS）
│   └── app_game_logic.c/.h        ← 推箱子游戏状态机（含路点执行引擎）
├── user/
│   ├── src/main.c                 ← 启动入口
│   └── src/isr.c                  ← PIT 中断（5ms / 20ms 任务分发）
├── mdk/                           ← MDK5 工程
└── iar/                           ← IAR 工程
```

## 快速上手

### 1. 初始化（main.c）— 已接入

```c
#include "chassis_pose_ctrl_call_example.h"
#include "chassis_menu.h"
#include "app_game_logic.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    // 1. 通信外设: OpenART 视觉串口
    uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);
    uart_rx_interrupt(UART_1, 1);

    // 2. IPS200 屏幕 + 按键 (调参菜单)
    ips200_set_dir(IPS200_CROSSWISE);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    key_init(10);

    // 3. 底盘初始化 + 调参菜单
    app_control_pipeline_init();
    chassis_menu_init();

    // 4. PIT 定时中断: 5ms 姿态采样 + 20ms 运动控制
    pit_ms_init(PIT_CH0, 5);
    pit_ms_init(PIT_CH1, 20);

    // 5. 主循环: 游戏状态机 + 调参菜单
    while (1) {
        Game_Logic_Task_Run();
        chassis_menu_task_10ms();
        chassis_menu_render_100ms();
    }
}
```

> **完整调用流程图** 见 `main.c` 文件头部注释的 ASCII 架构图。

### 1.1 姿态闭环调试（仅保留底盘 + 菜单，不跑游戏逻辑）

调试 IMU 航向角保持和轮速 PID 时，可以把主循环改为只跑菜单，不调用 `Game_Logic_Task_Run()`，然后用 `move_to_grid` 手动下发目标点：

```c
#include "chassis_pose_ctrl_call_example.h"
#include "chassis_menu.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    // IPS200 屏幕 + 按键
    ips200_set_dir(IPS200_CROSSWISE);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    key_init(10);

    // 底盘 + 菜单
    app_control_pipeline_init();
    chassis_menu_init();

    pit_ms_init(PIT_CH0, 5);
    pit_ms_init(PIT_CH1, 20);

    // ★ 调试: 下发一个固定目标网格点 (3, 5)，观察车模是否走直线到达
    app_control_pipeline_move_to_grid(3, 5);

    while (1) {
        chassis_menu_task_10ms();        // 按键调参保持可用
        chassis_menu_render_100ms();     // 屏幕实时显示位姿
    }
}
```

调试要点：

- **航向角保持**：下发目标后观察屏幕 `Yaw` 值，车模应沿直线行进而非画弧，若偏航说明 `Yaw Kp` 需要调大或 IMU 零偏未收敛（启动时确保车模静止 2 秒）。
- **定点精度**：到达后观察 `Pose X / Y` 与目标网格坐标 × 0.20m 的偏差，若偏差 > 2cm 可调 `Pos Kp`。
- **轮速 PID**：在菜单 PID 页面调节 `Wheel Kp/Ki/Kd`，改完即时生效，长按 K4 保存到 Flash。
- **多点移动测试**：到达后再手动改目标点，或写一个简单的序列：

```c
    // 多点移动调试示例
    app_control_pipeline_move_to_grid(3, 0);  // 先走到 (3,0)
    while (!app_control_pipeline_is_arrived()) {
        chassis_menu_task_10ms();
        chassis_menu_render_100ms();
    }
    app_control_pipeline_move_to_grid(3, 5);  // 再走到 (3,5)
    while (!app_control_pipeline_is_arrived()) {
        chassis_menu_task_10ms();
        chassis_menu_render_100ms();
    }
    // 到达后停车，可在此加断点观察最终位姿
    chassis_ctrl_stop();
    while (1) {
        chassis_menu_task_10ms();
        chassis_menu_render_100ms();
    }
```

### 1.2 里程计与视觉校正调试

当视觉定位（OpenART 副镜头）接入后，可在主循环中周期性校正里程计漂移：

```c
    // 视觉校正示例: 每次收到 OpenART 的绝对坐标后调用
    // 参数: 实际 x(m), 实际 y(m), 实际 yaw(°)
    app_control_pipeline_correct_pose(0.60f, 1.00f, 90.0f);

    // 读取当前位姿 (用于屏幕打印或逻辑判断)
    chassis_pose_t pose = chassis_ctrl_get_pose();
    // pose.x_m, pose.y_m, pose.yaw_deg
```

### 2. 中断调度（isr.c）— 已接入

```c
#include "chassis_pose_ctrl_call_example.h"

void PIT_IRQHandler(void)
{
    if (pit_flag_get(PIT_CH0)) {
        pit_flag_clear(PIT_CH0);
        app_control_pipeline_on_pit_5ms();   // 5ms: IMU 姿态采样 + 航向角积分
    }
    if (pit_flag_get(PIT_CH1)) {
        pit_flag_clear(PIT_CH1);
        app_control_pipeline_on_pit_20ms();  // 20ms: 编码器 + 里程计 + PID + 导航
    }
    __DSB();
}
```

> **PIT 中断原理**：`pit_ms_init(PIT_CH0, 5)` 将 PIT 通道 0 的倒计数器设为 `5ms × 75MHz = 375000`，到 0 时触发硬件中断。4 个通道共用 `PIT_IRQHandler` 入口，通过 `pit_flag_get()` 区分。函数名与启动文件向量表中的 `[WEAK]` 弱符号匹配，链接器自动用你的实现覆盖默认死循环。

### 3. 业务层调用

```c
// 让底盘移动到网格坐标 (3, 5)
app_control_pipeline_move_to_grid(3, 5);

// 轮询是否到达
if (app_control_pipeline_is_arrived()) {
    // 已到达，执行下一步
}

// 视觉重定位校正（可选）
app_control_pipeline_correct_pose(0.6f, 1.0f, 90.0f);
```

> IMU 零偏在 `chassis_imu_init()` 启动阶段自动静止标定，无需额外调用手动写入接口。

### 4. 2寸 IPS 二级调参菜单（新增）

在 `main` 主循环中周期调用：

```c
chassis_menu_task_10ms();
chassis_menu_render_100ms();
```

按键逻辑：

- 四个物理按键固定映射：
  - `KEY_1` = `C15`
  - `KEY_2` = `C14`
  - `KEY_3` = `C13`
  - `KEY_4` = `C12`

- 一级菜单（页面选择，按优先级排序）：`LIMIT -> NAV -> PID`
  - `K1` 上一页
  - `K2` 下一页
  - `K3` 进入二级参数页
  - `K4` 长按保存当前参数到 Flash
- 二级菜单（参数调节）：
  - `K1` 上一项
  - `K2` 下一项
  - `K3` 增大参数
  - `K4` 减小参数
  - `K1` 长按返回一级菜单
  - `K4` 长按保存当前参数到 Flash

Flash 持久化说明：

- 启动时自动从 Flash 读取参数并校验（magic/version/checksum）。
- 校验通过则覆盖当前调参值；失败则使用默认参数。
- 当前实现使用 `sector=127, page=7` 存储菜单参数，请确保该区域未被你的其他业务占用。

## 模块说明

| 模块 | 文件 | 功能 |
| ------ | ------ | ------ |
| **统一配置** | `chassis_config.h` | 所有引脚、物理尺寸、PID 增益、速度限幅等宏定义。**改硬件接线或调参只需改这一个文件。** |
| **PID 控制器** | `chassis_pid.c/.h` | 通用增量式 PID，提供 `init` / `step` / `reset` 三个接口。 |
| **电机驱动** | `chassis_motor.c/.h` | 封装 DRV8701E 的 PWM + DIR 控制，含方向修正系数。 |
| **编码器** | `chassis_encoder.c/.h` | 正交编码器 A/B 双通道读取，脉冲增量 → 轮速 (m/s)。 |
| **IMU 姿态** | `chassis_imu.c/.h` | IMU660RB SPI 读取，Z 轴陀螺积分 → 航向角，含零偏补偿。 |
| **麦轮运动学** | `chassis_mecanum.c/.h` | 正运动学（车体速度→四轮速度）、逆运动学（四轮→车体）、等比例限速。 |
| **顶层控制** | `chassis_ctrl.c/.h` | 整合以上模块：5ms 姿态采样 + 20ms 闭环控制 + 网格定点移动 + 航向保持。 |
| **调用层** | `chassis_pose_ctrl_call_example.c/.h` | `app_control_pipeline_*` 封装，隔离底盘实现，供 main/isr/上层业务调用。 |
| **BFS 纯寻路** | `algo_bfs_scout.c/.h` | ① `Algo_Nav_BFS` 点到点最短路径（不推箱，绕障碍行走）。② `Algo_Find_Nearest_Box_Observe_Point` 从玩家出发 BFS 扩散找最近箱子的侧面空地。供侦查阶段跑位 & 炸弹可达性检查使用。 |
| **推箱子求解器** | `algo_sokoban_solver.c/.h` | 地图分解法：将 N 个箱子拆为 N 个单箱子问题，逐个用 BFS（状态 = 玩家+箱子坐标）求解。支持阶段 1/2/3，含炸弹墙体搜索和路点压缩。 |
| **游戏状态机** | `app_game_logic.c/.h` | 完整比赛流程：侦查 → 观察 → 推箱/炸弹清障 → 完成。含路点导航引擎 `exec_push_waypoints()` 和炸弹推送子状态 `STAGE_3_BOMB_PUSH`。 |

### IMU 全局变量说明

- `car_angle` 是 IMU 模块导出的全局姿态变量（类型 `EulerAngle_t`，字段为 `roll/pitch/yaw`）。
- 定义位置：`project/code/chassis_imu.c`。
- 外部声明：`project/code/chassis_imu.h` 中 `extern EulerAngle_t car_angle;`。
- 推荐用法：控制层优先通过 `chassis_imu_get_yaw_deg()` 获取航向角，`car_angle` 主要用于调试和可视化查看，避免在业务层直接写入。

## 引脚分配一览

| 轮子 | PWM 引脚 | DIR 引脚 | 编码器 A 相 | 编码器 B 相 |
| ------ | ---------- | ---------- | ------------- | ------------- |
| 左前 (LF) | C11 | C10 | C0 | C1 |
| 右前 (RF) | C8 | C9 | C2 | C24 |
| 左后 (LB) | D3 | D2 | C3 | C25 |
| 右后 (RB) | C6 | C7 | B18 | B19 |

> 所有引脚定义在 `chassis_config.h` 中，修改宏即可切换引脚，无需改代码逻辑。  
> 若某轮转向相反，将对应 `CHASSIS_XX_DIR_SIGN` 改为 `-1.0f` 即可。

## 控制参数调整

打开 `chassis_config.h`，按需修改以下宏：

```c
// 车模物理尺寸（必须精确测量）
#define CHASSIS_WHEEL_RADIUS_M          (0.0315f)   // 轮半径 (m)
#define CHASSIS_HALF_WHEEL_BASE_M       (0.100f)   // 半轴距 (m)
#define CHASSIS_HALF_TRACK_WIDTH_M      (0.090f)   // 半轮距 (m)

// 编码器（根据型号和倍频方式确认）
#define CHASSIS_ENCODER_COUNTS_PER_REV  (1024.0f)  // 每转脉冲数

// 轮速 PID（先调 Kp，再加 Ki/Kd）
#define CHASSIS_WHEEL_PID_KP            (120.0f)
#define CHASSIS_WHEEL_PID_KI            (8.0f)
#define CHASSIS_WHEEL_PID_KD            (1.0f)

// 速度限幅
#define CHASSIS_MAX_LINEAR_SPEED_MPS    (0.35f)    // 最大平移速度 (m/s)
#define CHASSIS_MAX_WHEEL_SPEED_MPS     (0.60f)    // 单轮最大速度 (m/s)

// 里程计标定（走 1 格测实际距离后修正）
#define CHASSIS_ODOM_SCALE_X            (1.00f)
#define CHASSIS_ODOM_SCALE_Y            (1.00f)
```

## 内部控制流程

```text
5ms PIT 中断:
  IMU660RB → 读取 Z 轴角速度 → 减零偏 → 积分 → 航向角 yaw

20ms PIT 中断:
  ① 4 路编码器 → 脉冲增量 → 轮速 (m/s)
  ② 麦轮逆运动学 → 车体速度 (vx, vy)
  ③ 2D 旋转变换 → 全局速度 → 里程计积分 → 位姿 (x, y, yaw)
  ④ 导航 P 控制器 → 目标车体速度
  ⑤ 缓加速滤波 → 麦轮正运动学 → 四轮目标速度
  ⑥ 四路增量式 PID → PWM 输出 → 电机驱动
```

## 推箱子求解器

> 核心思路参考[逐飞演示车模浅析](https://mp.weixin.qq.com/s/bzW6Kdvn7R8pHLWfXLsfSg)中的"地图分解法"。

### 算法架构

```text
原始地图 (N 个箱子 + N 个目标)
  │
  ├─ 步骤 1: 地图分解  ─────────────────┐
  │   对 N 个箱子生成 N 张子地图            │
  │   当前箱子保留，其余箱子→墙，             │
  │   已完成箱子→空地，其余目标→空地          │
  │                                      ▼
  ├─ 步骤 2: 单箱 BFS 求解  ────── 状态 = (玩家行列, 箱子行列)
  │   状态空间 = 12×16×12×16 = 36864     共 ~108KB RAM
  │   BFS 找最短推送序列                    RT1064 解算 < 1ms
  │                                      │
  ├─ 步骤 3: 路点压缩  ──────────── 同方向连续移动合并为转弯点
  │   输出 SokoWaypointPath_t              │
  │                                      ▼
  └─ 步骤 4: 逐格导航  ──────────── move_to_grid 逐路点执行
```

### 三阶段支持

| 阶段 | 函数 | 策略 |
| --- | --- | --- |
| 第一阶段（基础模式） | `Sokoban_Solve_Stage1()` | 贪心分配：最近箱子 + 最近目标优先 |
| 第二阶段（分类模式） | `Sokoban_Solve_Stage2()` | 由 OpenART 识别后填充 `box_to_target_idx[]` 映射 |
| 第三阶段（策略模式） | `Sokoban_Find_Bomb_Wall()` + `Sokoban_Solve_Push_Bomb()` + `Sokoban_Apply_Bomb_Explosion()` | 找不可达目标 → 选最优墙体 → 推炸弹 → 爆破 → 重新推箱 |

### API 说明

```c
// ======== 第一阶段：任意箱→任意目标 ========
SokoFullSolution_t solution;
if (Sokoban_Solve_Stage1(g_game_map, player_pos, &solution)) {
    // solution.sub_solutions[0..N-1] 每个箱子的动作序列
    // solution.player_end_pos[0..N-1] 每个子解结束后玩家坐标
}

// ======== 第二阶段：指定映射 ========
uint8 mapping[3] = { 2, 0, 1 };  // 箱子0→目标2, 箱子1→目标0, 箱子2→目标1
Sokoban_Solve_Stage2(g_game_map, player_pos, mapping, 3, &solution);

// ======== 第三阶段：炸弹墙体搜索 + 推炸弹 ========
Point_t wall;
if (Sokoban_Find_Bomb_Wall(g_game_map, player_pos, blocked_target, &wall)) {
    // wall = 应推炸弹到此位置
    SokoActionSeq_t bomb_sol;
    if (Sokoban_Solve_Push_Bomb(g_game_map, player_pos, bomb_pos, wall, &bomb_sol)) {
        // bomb_sol.actions[] = 推炸弹动作序列, 转路点后逐格执行
        // 推完后:
        Sokoban_Apply_Bomb_Explosion(g_game_map, wall);  // 清除 3×3 内墙
        // 重新求解分类推箱 ...
    }
}

// ======== 动作序列 → 导航路点（只保留转弯点）========
SokoWaypointPath_t wp;
Sokoban_Actions_To_Waypoints(solution.sub_solutions[0].actions,
                             solution.sub_solutions[0].count,
                             player_pos, &wp);
// wp.points[0..wp.count-1] 即为车模逐点移动的目标坐标
```

### 状态机执行流程

```text
STAGE_PENDING_SCOUT ─→ 到最近箱子旁 ─→ 看有无贴图？
  ├─ 无贴图 → STAGE_1_BASIC_EXEC ──→ Sokoban_Solve_Stage1 → 逐路点执行 → STAGE_DONE
  └─ 有贴图 → STAGE_OBSERVE_ALL ──→ 遍历所有箱子建立映射
               ├─ 无炸弹 → STAGE_2_CLASS_EXEC ──→ Sokoban_Solve_Stage2 → STAGE_DONE
               └─ 有炸弹 → STAGE_3_STRATEGY_EXEC ─→ 尝试 Solve_Stage2
                            │ 成功: 逐路点推箱 → STAGE_DONE
                            │ 无解: 找不可达目标 → Find_Bomb_Wall → Solve_Push_Bomb
                            ▼
                     STAGE_3_BOMB_PUSH ─→ 逐路点推炸弹到墙 → Apply_Bomb_Explosion
                            │
                            └→ 回到 STAGE_3_STRATEGY_EXEC 重试 (循环直到全部推完 → STAGE_DONE)
```

#### 第三阶段炸弹全流程详解

`STAGE_3_STRATEGY_EXEC` 内部逻辑：

1. **尝试分类推箱** — 调用 `Sokoban_Solve_Stage2()` 忽略炸弹直接解
2. **若无解** — 说明有目标被内墙挡住：
   - 遍历地图所有 `MAP_TARGET`，用 `Algo_Nav_BFS()` 检查可达性
   - 找到第一个不可达目标 `blocked_target`
   - 调用 `Sokoban_Find_Bomb_Wall(map, player, blocked_target, &wall_pos)` 选最优爆破墙体
   - 在地图中找到 `MAP_BOMB` 的坐标
   - 调用 `Sokoban_Solve_Push_Bomb(map, player, bomb_pos, wall_pos, &action_seq)` 规划推炸弹路径
   - 转入 `STAGE_3_BOMB_PUSH` 子状态
3. **STAGE_3_BOMB_PUSH** — 按路点逐格推炸弹到目标墙体，完成后：
   - `Sokoban_Apply_Bomb_Explosion(g_game_map, wall_pos)` 清除 3×3 内墙
   - 清除炸弹原格为空地
   - 重置 `g_soko_exec_init`，回到 `STAGE_3_STRATEGY_EXEC` 重新尝试分类推箱
4. **循环**：炸墙后路通了，`Sokoban_Solve_Stage2()` 成功 → 正常推箱 → `STAGE_DONE`

路点执行引擎 `exec_push_waypoints()` 在 `app_game_logic.c` 中统一处理：

- 自动生成当前子解路点 → 逐点调用 `move_to_grid` → 到达后切换下一路点
- 当前子解走完自动切换下一个箱子的子解 → 全部完成返回 1

### `algo_bfs_scout` 与 `algo_sokoban_solver` 的分工

| 模块 | 功能 | 使用场景 |
| --- | --- | --- |
| `algo_bfs_scout` | **纯行走寻路**（不考虑推箱）+ 最近箱子观察点搜索 | 侦查跑位 `STAGE_PENDING_SCOUT`、第三阶段炸弹可达性检查 |
| `algo_sokoban_solver` | **推箱子求解**（状态含玩家+箱子）+ 路点压缩 + 炸弹墙体搜索 | 三个推箱执行阶段 |

两者互补：scout 负责"人不推箱的快速跑位"，solver 负责"完整推箱路径规划"。solver 内部的 `Sokoban_Find_Bomb_Wall()` 也调用了 scout 的 `Algo_Nav_BFS()` 来做可达性判断。

### 桌面调试

用逐飞推荐的 [SokoPlayer HTML5](https://sokoban.cn/sokoplayer/SokoPlayer_HTML5.php) 在线验证：

1. 将 `g_game_map` 转为标准推箱子地图字符（`#`=墙, ` `=空地, `$`=箱子, `.`=目标, `@`=玩家）
2. 将 `sub_solutions[i].actions[]` 转为 UDLR 方向字符串
3. 粘贴到网站验证路径是否正确

## 编译与烧录

- **IAR**: 打开 `project/iar/rt1064.eww`，`code` 分组已配置好全部源文件，**直接编译即可**。
- **MDK5**: 打开 `project/mdk/rt1064.uvprojx`，需手动将下方 `.c` 文件添加到工程 Source Group。

需要编译的 `.c` 文件清单：

| 文件 | 说明 |
| ------ | ------ |
| `chassis_ctrl.c` | 底盘顶层控制 |
| `chassis_motor.c` | 电机驱动 |
| `chassis_encoder.c` | 编码器测速 |
| `chassis_imu.c` | IMU 姿态 |
| `chassis_mecanum.c` | 麦轮运动学 |
| `chassis_pid.c` | PID 控制器 |
| `chassis_pose_ctrl_call_example.c` | 调用层封装 |
| `chassis_menu.c` | IPS 调参菜单 |
| `algo_bfs_scout.c` | BFS 纯寻路 + 最近箱子观察点 |
| `algo_sokoban_solver.c` | 推箱子求解器 |
| `app_game_logic.c` | 游戏状态机 |

> `chassis_pose_ctrl.c` 已清空，**不参与编译**。仅保留 `.h` 头文件做旧 API 兼容映射。

## 当前进度

- [x] 底盘控制链路模块化拆分（7 个独立模块）
- [x] main.c / isr.c 接入 5ms + 20ms 调度
- [x] PIT 定时器初始化配置
- [x] IAR 工程编译清单更新
- [x] 游戏状态机主循环接入
- [x] `g_player_pos` 从里程计实时同步
- [x] MAP_ROWS/MAP_COLS 修正为 12行×16列
- [x] IMU 模块消除硬编码魔数、修复 `invSqrt` UB、消除宏名冲突
- [x] 推箱子求解器实现（地图分解法 + 单箱 BFS + 路点导航）
- [x] 游戏状态机一/二/三阶段推箱执行逻辑接入
- [x] 第三阶段炸弹全流程：推炸弹 → 爆破 → 重新求解完整链路
- [x] main.c 完整调用流程文档化

## 后续待办

1. **MDK 工程更新**：将新 `.c` 文件加入 MDK 编译清单（IAR 已完成）
2. **实车标定**：编码器脉冲数、轮径、PID 参数、方向极性、里程计比例（参数全在 `chassis_config.h`）
3. **对接 OpenART 视觉串口**：替换 `HAL_VISION_GET_BOX_CLASS_ID()` 占位宏
4. **分类映射表填充**：在 `STAGE_OBSERVE_ALL` 阶段由 OpenART 识别结果写入 `g_box_to_target[]`
