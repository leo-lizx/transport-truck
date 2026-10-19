/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 RT1064DVL6A 开源库的一部分
*
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "clock_config.h"
#include "chassis_ctrl.h"
#include "chassis_pid.h"
#include "app_suction_test.h"
#include <stdio.h>
#include "fsl_rtwdog.h"  /* 2s 硬件看门狗 */

#if (DEBUG_UART_ENABLE == 0)
/* 无线调试串口拆除后不再格式化日志，避免无输出设备时浪费主循环时间。 */
#define printf(...) ((void)0)
#endif

/* 编译期运行模式，保留原编号。默认进入吸盘台架测试。 */
#define MAIN_RUN_MODE_YAW_HOLD        (1)   /* 航向保持调试 */
#define MAIN_RUN_MODE_SINGLE_WHEEL    (2)   /* 单轮 PID 调试 */
#define MAIN_RUN_MODE_POINT_NAV       (3)   /* 预设航点导航 */
#define MAIN_RUN_MODE_SUCTION_TEST    (4)   /* C15 按住吸附，松手泄压 */

/* 修改此行切换模式：YAW_HOLD / SINGLE_WHEEL / POINT_NAV / SUCTION_TEST。 */
#define MAIN_RUN_MODE                 (MAIN_RUN_MODE_SUCTION_TEST)

static volatile uint32 s_main_tick_pending = 0U;   /* PIT_CH0 累加, 主循环消费 */
static volatile uint32 s_main_tick_overrun = 0U;   /* 一次消费 >1 的累计差值 */
static volatile uint32 s_main_tick_total   = 0U;   /* 主循环已消费的 tick 总数 */

/* 由 isr.c 的 PIT_CH0 分支每 5ms 调用。 */
void main_loop_on_pit_tick(void)
{
    s_main_tick_pending++;
}

/* 等待并消费 5ms 节拍；合并节拍计入 overrun，主循环仍只执行一轮任务。
 * 保留原忙等方式，避免改变 SWD 调试行为；读取与清零在临界区中完成。 */
static uint32 wait_for_tick(void)
{
    uint32 ticks;
    while (s_main_tick_pending == 0U)
    {
    }
    __disable_irq();
    ticks = s_main_tick_pending;
    s_main_tick_pending = 0U;
    __enable_irq();
    if (ticks > 1U)
    {
        s_main_tick_overrun += (ticks - 1U);
    }
    s_main_tick_total += ticks;
    return ticks;
}

/* 保留原测试起点和网格换算；新赛道坐标不在本次清理范围内。 */
#define MAIN_POS_NAV_START_X_GRID     (1.0f)
#define MAIN_POS_NAV_START_Y_GRID     (5.5f)
#define MAIN_POS_GRID_TO_M_X(g)       (((float)(g) - 0.5f) * CHASSIS_GRID_STEP_X_M)
#define MAIN_POS_GRID_TO_M_Y(g)       (((float)(g) - 0.5f) * CHASSIS_GRID_STEP_Y_M)

#if (MAIN_RUN_MODE == MAIN_RUN_MODE_YAW_HOLD)
#define MAIN_POS_NAV_HOLD_YAW_DEG     (180.0f)  /* 保留原航向保持目标 180° */
#endif

#if (MAIN_RUN_MODE == MAIN_RUN_MODE_POINT_NAV)
#define MAIN_POINT_NAV_WARMUP_TICKS   (200U)   /* 200 × 5ms = 1s */
#endif

#if (MAIN_RUN_MODE == MAIN_RUN_MODE_SINGLE_WHEEL)
#define MAIN_PID_DEBUG_WHEEL_INDEX    (CHASSIS_WHEEL_LF)  /* 0=LF, 1=RF, 2=LB, 3=RB */
#define MAIN_PID_DEBUG_TARGET_MPS     (0.10f)             /* target wheel speed, m/s */
#define MAIN_PID_DEBUG_FORCE_PID      (1)                 /* 1=use KP/KI/KD below */
#define MAIN_PID_DEBUG_KP             (200.0f)
#define MAIN_PID_DEBUG_KI             (50.0f)
#define MAIN_PID_DEBUG_KD             (10.0f)
#endif /* MAIN_RUN_MODE_SINGLE_WHEEL */

#if (MAIN_RUN_MODE == MAIN_RUN_MODE_SINGLE_WHEEL) && (1 == MAIN_PID_DEBUG_FORCE_PID)
/*
 * 把 main 内 KP/KI/KD 写进 chassis_tune_params, 仅覆盖被调试的那一个轮子,
 * 其余轮子的 PID 维持编译期初始化值不动. 在 chassis_ctrl_init 之后调用.
 */
static void main_apply_debug_wheel_pid(void)
{
    chassis_tune_params_t tune_params;
    uint8 idx = (uint8)MAIN_PID_DEBUG_WHEEL_INDEX;

    chassis_ctrl_get_tune_params(&tune_params);
    tune_params.wheel_pid_kp[idx] = MAIN_PID_DEBUG_KP;
    tune_params.wheel_pid_ki[idx] = MAIN_PID_DEBUG_KI;
    tune_params.wheel_pid_kd[idx] = MAIN_PID_DEBUG_KD;
    chassis_ctrl_set_tune_params(&tune_params);
}
#endif

int main(void)
{
    clock_init(BOARD_BOOTCLOCKRUN_CORE_CLOCK);
#if DEBUG_UART_ENABLE
    debug_init();
#endif

#if (MAIN_RUN_MODE == MAIN_RUN_MODE_SUCTION_TEST)
    /* 先发送两路关闭指令，再进行底盘 IMU 标定。 */
    app_suction_test_init();
#endif

    /* 初始化前保持车体静止，供 IMU 零偏标定使用。 */
    chassis_ctrl_init();
    chassis_ctrl_set_pose(MAIN_POS_GRID_TO_M_X(MAIN_POS_NAV_START_X_GRID),
                          MAIN_POS_GRID_TO_M_Y(MAIN_POS_NAV_START_Y_GRID),
                          0.0f);

#if (MAIN_RUN_MODE == MAIN_RUN_MODE_SINGLE_WHEEL)
    /* 单轮闭环由 PIT_CH1 的 20ms 底盘任务执行。 */
  #if (1 == MAIN_PID_DEBUG_FORCE_PID)
    main_apply_debug_wheel_pid();
  #endif
    chassis_ctrl_start_single_wheel_pid_debug((uint8)MAIN_PID_DEBUG_WHEEL_INDEX,
                                              MAIN_PID_DEBUG_TARGET_MPS);
    printf("SW_BOOT wheel=%d target=%.3f kp=%.1f ki=%.1f kd=%.1f\n",
           (int)MAIN_PID_DEBUG_WHEEL_INDEX,
           MAIN_PID_DEBUG_TARGET_MPS,
           MAIN_PID_DEBUG_KP,
           MAIN_PID_DEBUG_KI,
           MAIN_PID_DEBUG_KD);
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_YAW_HOLD)
    chassis_ctrl_hold_yaw(MAIN_POS_NAV_HOLD_YAW_DEG);
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_POINT_NAV)
    /* 暖机期间保持初始航向，随后由航点状态机下发目标。 */
    chassis_ctrl_hold_yaw(0.0f);
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_SUCTION_TEST)
    chassis_ctrl_stop();
#endif

    /* CH0: 5ms 姿态采样与主循环节拍；CH1: 20ms 底盘闭环。 */
    pit_ms_init(PIT_CH0, 5);
    pit_ms_init(PIT_CH1, 20);

    /* 保留原看门狗参数：主循环停止刷新约 2s 后复位。 */
    {
        rtwdog_config_t wdt_cfg;
        RTWDOG_GetDefaultConfig(&wdt_cfg);
        wdt_cfg.enableRtwdog   = true;
        wdt_cfg.prescaler      = kRTWDOG_ClockPrescalerDivide256;
        wdt_cfg.timeoutValue   = 256U;
        wdt_cfg.enableUpdate   = true;
        wdt_cfg.workMode.enableDebug = true;
        RTWDOG_Init(RTWDOG, &wdt_cfg);
    }

    while (1)
    {
        (void)wait_for_tick();
        RTWDOG_Refresh(RTWDOG);

    #if (MAIN_RUN_MODE == MAIN_RUN_MODE_SINGLE_WHEEL)
        /* 内部每 100ms 输出目标轮速与实际轮速。 */
        chassis_pid_debug_task_5ms();
    #elif (MAIN_RUN_MODE == MAIN_RUN_MODE_YAW_HOLD)
        /* 内部每 50ms 输出航向调试数据。 */
        chassis_ctrl_attitude_debug_task_5ms();
    #elif (MAIN_RUN_MODE == MAIN_RUN_MODE_SUCTION_TEST)
        app_suction_test_task(s_main_tick_total * 5U);
    #elif (MAIN_RUN_MODE == MAIN_RUN_MODE_POINT_NAV)

        /* 路径航点遍历状态机:
         *   暖机 1s → 依次下发航点 → 全程结束后驻停于终点.
         *
         *   标志位: chassis_ctrl_is_arrived() (chassis 内部 s_arrived)
         *     == 0: 行驶中 / 旋转中 (发车时由 move_to_grid / rotate_to_deg 内部清零)
         *     == 1: 已到达当前目标
         *   到达后立即更新索引并派发下一个航点, 自动将标志位清零; 全程结束驻停. */
        {
            /* 航点描述: { x(格), y(格), yaw(°) }
             *   x/y = 网格坐标, 运行时自动换算为米
             *   yaw = 传给 chassis_ctrl_move_to_m() 的目标航向角 */
            typedef struct { float x; float y; float yaw; } s_nav_wp_t;

            /* ═══════════ 航点序列 — 改这里 ═══════════ */
            static const s_nav_wp_t s_wps[] = {
                /*    x格    y格    yaw°   说明 */
                {   1.0f, 10.0f,  0.0f },
                {  14.0f, 10.0f,  0.0f },
                {  14.0f,  1.0f,  0.0f },
                {   1.0f,  1.0f,  0.0f },

                {   1.0f, 3.0f,  0.0f },
                {  14.0f, 3.0f,  0.0f },
                {  14.0f,  5.0f,  0.0f },
                {   1.0f,  5.0f,  0.0f },

                {   1.0f, 7.0f,  0.0f },
                {  14.0f, 7.0f,  0.0f },
                {  14.0f,  9.0f,  0.0f },
                {   1.0f,  9.0f,  0.0f },

                {   1.0f, 10.0f,  0.0f },
                {  14.0f, 10.0f,  0.0f },
                {  14.0f,  1.0f,  0.0f },
                {   1.0f,  1.0f,  0.0f },

                {  11.0f, 1.0f,  0.0f },
                {  11.0f, 10.0f,  0.0f },
                {  9.0f,  10.0f,  0.0f },
                {   9.0f,  1.0f,  0.0f },

                {  1.0f, 2.0f,  0.0f },
                {  1.0f, 3.0f,  0.0f },
                {  1.0f, 4.0f,  0.0f },
                {  2.0f, 4.0f,  0.0f },

                {  3.0f, 4.0f,  0.0f },
                {  10.0f, 3.0f,  0.0f },
                {  1.0f, 4.0f,  0.0f },
                {  2.0f, 4.0f,  0.0f },

                {  1.0f, 5.50f,  0.0f },

            };

            static uint8  s_wp_idx       = 0U;   /* 当前航点索引 (0 起, 到 count 停止) */
            static uint16 s_warmup_ticks = 0U;   /* 暖机计数 (5ms tick) */
            static uint8  s_nav_started  = 0U;   /* 0=暖机中, 1=遍历进行中 */

#define S_NAV_DISPATCH(wp)                                                   \
    chassis_ctrl_move_to_m(MAIN_POS_GRID_TO_M_X((wp)->x),                   \
                           MAIN_POS_GRID_TO_M_Y((wp)->y),                   \
                           (wp)->yaw)

            if (!s_nav_started) {
                /* 暖机: 等待 IMU KF + 编码器 LPF + 静止窗口全部就绪 */
                if (++s_warmup_ticks >= MAIN_POINT_NAV_WARMUP_TICKS) {
                    s_nav_started = 1U;
                    /* 派发首个航点; chassis 内部置 s_arrived=0 → 标志位: 未到 */
                    S_NAV_DISPATCH(&s_wps[0]);
                }
            } else if (s_wp_idx < (uint8)(sizeof(s_wps) / sizeof(s_wps[0]))) {
                /* chassis_ctrl_is_arrived() == 1 → 标志位: 已到 → 遍历下一个 */
                if (chassis_ctrl_is_arrived()) {
                    s_wp_idx++;   /* 更新索引 */
                    if (s_wp_idx < (uint8)(sizeof(s_wps) / sizeof(s_wps[0]))) {
                        /* 派发下一航点; chassis 内部清零 s_arrived → 标志位: 未到 */
                        S_NAV_DISPATCH(&s_wps[s_wp_idx]);
                    }
                    /* s_wp_idx == count: 全程结束, 车驻停于终点 (1,5.5) */
                }
            }

#undef S_NAV_DISPATCH

#if DEBUG_UART_ENABLE
            /* 调试串口开启时每 50ms 读取并打印，关闭时不计算日志数据。 */
            {
                static uint8 s_pos_div = 0U;
                if (++s_pos_div >= 10U) {
                    chassis_pose_t pose;
                    float tgt_x, tgt_y, dx, dy, dist_sq;
                    s_pos_div = 0U;
                    pose = chassis_ctrl_get_pose();
                    chassis_ctrl_get_point_nav_target_m(&tgt_x, &tgt_y);
                    dx      = tgt_x - pose.x_m;
                    dy      = tgt_y - pose.y_m;
                    dist_sq = dx * dx + dy * dy;
                    printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%lu\n",
                           pose.x_m, pose.y_m,
                           tgt_x, tgt_y,
                           dist_sq,
                           pose.yaw_deg,
                           (int)chassis_ctrl_is_arrived(),
                           (int)s_wp_idx,
                           (unsigned long)g_chassis_arrival_count);
                }
            }
#endif
        }
    #endif
    }
}
