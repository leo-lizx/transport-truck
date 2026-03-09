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
│   ├── algo_bfs_scout.c/.h        ← BFS 路径搜索算法
│   └── app_game_logic.c/.h        ← 推箱子游戏状态机
├── user/
│   ├── src/main.c                 ← 启动入口
│   └── src/isr.c                  ← PIT 中断（5ms / 20ms 任务分发）
├── mdk/                           ← MDK5 工程
└── iar/                           ← IAR 工程
```

## 快速上手

### 1. 初始化（main.c）

```c
#include "chassis_pose_ctrl_call_example.h"

int main(void)
{
    board_init();                    // 逐飞板级初始化
    app_control_pipeline_init();     // 初始化底盘（IMU + 编码器 + 电机 + PID）
    // ... 其他初始化 ...
    while (1) { /* 主循环 */ }
}
```

### 2. 中断调度（isr.c）

```c
#include "chassis_pose_ctrl_call_example.h"

// 在 5ms PIT 中断中调用：
app_control_pipeline_on_pit_5ms();   // IMU 姿态采样 + 航向角积分

// 在 20ms PIT 中断中调用：
app_control_pipeline_on_pit_20ms();  // 编码器 → 里程计 → 导航 → PID → 电机
```

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

// 陀螺零偏标定（可选，开机静止采样后写入）
app_control_pipeline_set_gyro_bias(0.15f);
```

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
#define CHASSIS_WHEEL_RADIUS_M          (0.030f)   // 轮半径 (m)
#define CHASSIS_HALF_WHEEL_BASE_M       (0.080f)   // 半轴距 (m)
#define CHASSIS_HALF_TRACK_WIDTH_M      (0.070f)   // 半轮距 (m)

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

## 编译与烧录

- **MDK5**: 打开 `project/mdk/rt1064.uvprojx`，需将 `project/code/` 下的 `.c` 文件添加到工程 Source Group
- **IAR**: 打开 `project/iar/rt1064.eww`，同样需要将新 `.c` 文件加入编译列表

需要编译的 `.c` 文件清单：

- `chassis_ctrl.c`
- `chassis_motor.c`
- `chassis_encoder.c`
- `chassis_imu.c`
- `chassis_mecanum.c`
- `chassis_pid.c`
- `chassis_pose_ctrl_call_example.c`
- `algo_bfs_scout.c`
- `app_game_logic.c`

> `chassis_pose_ctrl.c` 已清空，可从编译列表中移除。

## 后续待办

1. 在 `main.c` / `isr.c` 中实际接入 `app_control_pipeline_*` 调度
2. 将新 `.c` 文件加入 MDK/IAR 工程编译清单
3. 实车标定：编码器脉冲数、轮径、PID 参数、方向极性、里程计比例
4. 对接 OpenART 视觉串口，替换 `HAL_VISION_GET_BOX_CLASS_ID()` 占位宏
