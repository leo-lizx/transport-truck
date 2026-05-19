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
#include "chassis_ctrl.h"
#include "chassis_pid.h"
#include "chassis_menu.h"
#include "app_game_logic.h"
#include "app_link.h"   /* P0-1: 视觉-主控帧协议 */
#include "algo_sokoban_solver.h"
#include <stdarg.h>
#include <math.h>       /* sqrtf — POINT_NAV 调试打印用 */
#include <stdio.h>

/*==========================================================================
 *  P0-5: 主循环 5ms tick 节拍 (替代 system_delay_ms 阻塞)
 *  - PIT_CH0 ISR 每 5ms 调用 main_loop_on_pit_tick(), 累加 s_main_tick_pending
 *  - 主循环用 wait_for_tick() 消费节拍, 期间 __WFI 进入低功耗等待中断唤醒
 *  - 当一次消费的 tick > 1 表示主循环上一轮耗时 >5ms, 记入 overrun 仅观测不丢拍
 *  - 三个计数器均为 file-static volatile, 仅 IPS / Live Watch 调试可见
 *========================================================================*/
static volatile uint32 s_main_tick_pending = 0U;   /* PIT_CH0 累加, 主循环消费 */
static volatile uint32 s_main_tick_overrun = 0U;   /* 一次消费 >1 的累计差值 */
static volatile uint32 s_main_tick_total   = 0U;   /* 主循环已消费的 tick 总数 */

/* 由 isr.c 中 PIT_CH0 分支调用; 用 extern 声明对外可见, 实现见本文件末 */
void main_loop_on_pit_tick(void)
{
    s_main_tick_pending++;
}

/*
 * 阻塞直到下一个 5ms tick 到来, 返回本次消费的 tick 数 (>=1).
 * - 没有 pending tick 时 __WFI() 让 CPU 休眠, 任意中断 (PIT/UART/SysTick) 可唤醒
 * - 唤醒后 while 兜底再判一次, 防止 WFI 偶发未睡稳或被无关中断唤醒
 * - 「读 + 清零」用 __disable_irq/__enable_irq 包成临界区, 与 PIT_CH0 ISR 互斥
 */
static uint32 wait_for_tick(void)
{
    uint32 ticks;
    while (s_main_tick_pending == 0U)
    {
        /* __WFI();  P0-5: 临时关闭, 避免 SWD 进 WFI 后断连; 稳定后再开 */
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

/*===========================================================================
 *  顶层调试模式开关 (三选一, 上电只能进一种状态)
 *    MAIN_RUN_MODE_GAME           : 游戏状态机 (推箱子) 完整运行
 *    MAIN_RUN_MODE_YAW_HOLD       : 航向保持测试 (车体不动, IMU 闭环只调 wz)
 *    MAIN_RUN_MODE_SINGLE_WHEEL   : 单轮 PID 调试 (只给一个轮子目标速度)
 *    MAIN_RUN_MODE_OPENART1_TEST  : OpenART1 -> UART4 地图链路测试 (不驱动车体)
 *=========================================================================*/
#define MAIN_RUN_MODE_GAME            (0)
#define MAIN_RUN_MODE_YAW_HOLD        (1)
#define MAIN_RUN_MODE_SINGLE_WHEEL    (2)
#define MAIN_RUN_MODE_POINT_NAV       (3)
#define MAIN_RUN_MODE_SOKO_SELFTEST   (4)
#define MAIN_RUN_MODE_OPENART1_TEST   (5)

/* >>>>>>>>>>>> 改这里切换调试模式 <<<<<<<<<<<< */
#define MAIN_RUN_MODE                 (MAIN_RUN_MODE_POINT_NAV)  /* 电机低速验证 */
/* <<<<<<<<<<<< 改这里切换调试模式 >>>>>>>>>>>> */

/* OpenART1 地图链路硬件口: 若实测 UART4 走 D0/D1, 只改下面两行宏. */
#define MAIN_OPENART1_UART            (UART_4)
#define MAIN_OPENART1_UART_TX         (UART4_TX_C16)
#define MAIN_OPENART1_UART_RX         (UART4_RX_C17)

/* OpenART1 测试日志口: 接 DAP 下载器虚拟串口/USB-TTL, 只用于本轮链路观测. */
#define MAIN_TEST_LOG_UART            (UART_1)
#define MAIN_TEST_LOG_UART_TX         (UART1_TX_B12)
#define MAIN_TEST_LOG_UART_RX         (UART1_RX_B13)

#if (MAIN_RUN_MODE == MAIN_RUN_MODE_SOKO_SELFTEST)
/*
 * 固定自测图: 由用户编辑的竖版地图整理为 12x16 横版格式.
 * 目的: 保留原始关卡结构, 同时满足求解器固定 12 行 16 列的输入约束.
 */
static const char s_soko_selftest_map[MAP_ROWS][MAP_COLS + 1] = {
    "################",
    "#----------#---#",
    "#----------##@-#",
    "#----$------#--#",
    "#--------------#",
    "#---$-##-------#",
    "#-----#-##-----#",
    "#--$--#--------#",
    "#-----#--------#",
    "#-----#---#----#",
    "#-----#---##...#",
    "################"
};

static uint8 main_selftest_char_to_map(char ch)
{
    switch (ch)
    {
    case '#': return MAP_WALL;
    case '-': return MAP_EMPTY;
    case '.': return MAP_TARGET;
    case '$': return MAP_BOX;
    case '*': return MAP_BOMB;
    case '@': return MAP_EMPTY;
    default:  return MAP_EMPTY;
    }
}

static char main_selftest_action_to_char(SokoAction_e act)
{
    switch (act)
    {
    case SOKO_ACT_UP:    return 'U';
    case SOKO_ACT_DOWN:  return 'D';
    case SOKO_ACT_LEFT:  return 'L';
    case SOKO_ACT_RIGHT: return 'R';
    default:             return '?';
    }
}

static void main_selftest_log(const char *fmt, ...)
{
    char buffer[192];
    va_list args;
    int len;

    va_start(args, fmt);
    len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    if (len <= 0)
    {
        return;
    }

    printf("%s", buffer);

    if (len > (int)(sizeof(buffer) - 1U))
    {
        len = (int)(sizeof(buffer) - 1U);
    }
    uart_write_buffer(UART_1, (const uint8 *)buffer, (uint32)len);
}

static void main_selftest_print_map_and_player(Point_t player_pos)
{
    uint8 r;

    main_selftest_log("MAP_ROWS=%d\n", (int)MAP_ROWS);
    main_selftest_log("MAP_COLS=%d\n", (int)MAP_COLS);
    main_selftest_log("PLAYER_START=%d,%d\n", (int)player_pos.x, (int)player_pos.y);
    main_selftest_log("MAP_BEGIN\n");
    for (r = 0U; r < MAP_ROWS; r++)
    {
        main_selftest_log("%s\n", s_soko_selftest_map[r]);
    }
    main_selftest_log("MAP_END\n");
}

static void main_run_soko_selftest_once(void)
{
    uint8 map[MAP_ROWS][MAP_COLS];
    Point_t player_pos = {-1, -1};
    SokoFullSolution_t solution;
    uint8 r, c;

    for (r = 0U; r < MAP_ROWS; r++)
    {
        for (c = 0U; c < MAP_COLS; c++)
        {
            char ch = s_soko_selftest_map[r][c];
            if ('@' == ch)
            {
                player_pos.x = (int8)c;
                player_pos.y = (int8)r;
            }
            map[r][c] = main_selftest_char_to_map(ch);
        }
    }

    main_selftest_log("SOKO_MAP=T4\n");
    main_selftest_print_map_and_player(player_pos);
    if ((player_pos.x < 0) || (player_pos.y < 0))
    {
        main_selftest_log("SOLVED=0\n");
        main_selftest_log("ERR=NO_PLAYER\n");
        return;
    }

    memset(&solution, 0, sizeof(solution));
    if (!Sokoban_Solve_Stage1(map, player_pos, &solution))
    {
        main_selftest_log("SOLVED=0\n");
        main_selftest_log("ERR=NO_SOLUTION\n");
        return;
    }

    main_selftest_log("SOLVED=%d\n", (int)solution.is_solved);
    main_selftest_log("BOXES=%d\n", (int)solution.total_boxes);
    for (r = 0U; r < solution.total_boxes; r++)
    {
        uint16 i;
        main_selftest_log("SEG%d_COUNT=%d\n", (int)r, (int)solution.sub_solutions[r].count);
        main_selftest_log("SEG%d_PATH=", (int)r);
        for (i = 0U; i < solution.sub_solutions[r].count; i++)
        {
            main_selftest_log("%c", main_selftest_action_to_char(solution.sub_solutions[r].actions[i]));
        }
        main_selftest_log("\n");
        main_selftest_log("PLAYER_END=%d,%d\n",
                          (int)solution.player_end_pos[r].x,
                          (int)solution.player_end_pos[r].y);
    }
}

static void main_run_soko_selftest_periodic_5ms(void)
{
    static uint16 s_div = 0U;

    s_div++;
    if (s_div >= 200U)
    {
        s_div = 0U;
        main_selftest_log("===SOKO_SELFTEST_ALIVE===\n");
        main_run_soko_selftest_once();
    }
}
#endif

/* 航向保持目标角 (deg), 仅 YAW_HOLD 模式生效 */
#define MAIN_HOLD_YAW_TARGET_DEG      (0.0f)

/* ============================================================
 * 位置闭环坐标系 (POINT_NAV 模式) — 整数格约定
 *   单位: 1 格步长 (X = 3.2m / 14 ≈ 0.2286m, Y = 2.4m / 10 = 0.24m)
 *   范围: X ∈ [0, 14], Y ∈ [0, 10]   (0 = 左/上边界, 14/10 = 右/下边界)
 *   起点: (0.5, 5.5) 格 — 发车区中心, 距左墙半格, 距上墙 5.5 格
 *
 *   目标坐标传整数即可, 内部按 grid * step 自动换算成米送入闭环.
 *   起点用半整数 (0.5, 5.5) 描述发车区中心位置, 与硬件实测吻合.
 * ============================================================ */
#define MAIN_POS_NAV_START_X_GRID     (1.0f)
#define MAIN_POS_NAV_START_Y_GRID     (5.5f)

/* >>>>>>>>>>>> 改这两行换目标格 <<<<<<<<<<<< */
#define MAIN_POS_NAV_TARGET_X_GRID    (14)     /* 整数 0..14, 14 = 右边界 */
#define MAIN_POS_NAV_TARGET_Y_GRID    (10)     /* 整数 0..10, 10 = 下边界 */
/* <<<<<<<<<<<< 改这两行换目标格 >>>>>>>>>>>> */

#define MAIN_POS_NAV_HOLD_YAW_DEG     (0.0f)   /* 全程锁住 0° 航向 */

/*
 * 上电暖机等待时长 (5ms tick 数). 200 × 5ms = 1s.
 * 等待以下子系统稳定后, 主循环才下发第1个导航目标:
 *   1. IMU KF 收敛 (~500ms): Kalman P 压缩到稳态, 零偏估计准确;
 *      未收敛时 yaw 漂移 1~5°, 里程计误差随时间线性累积.
 *   2. 编码器 LPF 稳定 (~100ms): 前几拍速度反馈偏大, odometry 的 vx/vy 有初始误差.
 *   3. 静止检测滑窗填满 (STILL_WINDOW_LEN × 5ms): 窗口未满时在线零偏自适应不工作.
 * 暖机期 chassis_ctrl 保持 YAW_HOLD, 位置积分 s_pos_i 不运行, 无积分超调风险.
 */
#define MAIN_POINT_NAV_WARMUP_TICKS   (200U)   /* 200 × 5ms = 1s 暖机 */

/* 格 → 米换算 (POINT_NAV 专用, 不影响 BFS 的内场索引体系).
 * 约定: 整数 n = 第 n 格中心 (1-based), 公式为 (n - 0.5) × STEP.
 *   n=1  → 0.5 × STEP (第1格中心, 距左/上墙半格)
 *   n=14 → 13.5 × STEP_X = 3.086m (第14格中心, 距右墙 0.114m, 可到达)
 *   n=10 → 9.5  × STEP_Y = 2.28m  (第10格中心, 距下墙 0.12m,  可到达)
 * 注: 若传入半整数 5.5 则 (5.5-0.5)×STEP = 5×STEP = 第5/6格边界, 用于起点描述. */
#define MAIN_POS_GRID_TO_M_X(g)       (((float)(g) - 0.5f) * CHASSIS_GRID_STEP_X_M)
#define MAIN_POS_GRID_TO_M_Y(g)       (((float)(g) - 0.5f) * CHASSIS_GRID_STEP_Y_M)
#define MAIN_POS_NAV_TARGET_X_M       MAIN_POS_GRID_TO_M_X(MAIN_POS_NAV_TARGET_X_GRID)
#define MAIN_POS_NAV_TARGET_Y_M       MAIN_POS_GRID_TO_M_Y(MAIN_POS_NAV_TARGET_Y_GRID)

/* ========================================================================== */
/*  ⬇⬇⬇ 以下是单轮 PID 调试专用代码, 仅在 MAIN_RUN_MODE_SINGLE_WHEEL 生效 ⬇⬇⬇  */
/*  ⬇⬇⬇ 姿态闭环调试阶段这里全部被 #if 屏蔽, 不会被编译, 不要删 ⬇⬇⬇          */
/* ========================================================================== */
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_SINGLE_WHEEL)
/*---------------------------------------------------------------------------
 * 单轮 PID 调试参数 (仅 SINGLE_WHEEL 模式生效)
 *   WHEEL_INDEX : 0=LF 1=RF 2=LB 3=RB
 *   TARGET_MPS  : 阶跃目标线速度 (m/s), 上限受 debug_target_speed_clamp() 约束
 *   FORCE_PID   : 1=用下面 KP/KI/KD 覆盖 menu 参数 (仅覆盖被调试那一轮)
 *                 0=沿用 menu/Flash 中的 PID
 *-------------------------------------------------------------------------*/
#define MAIN_PID_DEBUG_WHEEL_INDEX    (CHASSIS_WHEEL_LF)  /* 左前轮, 最容易观察 */
#define MAIN_PID_DEBUG_TARGET_MPS     (3.3f)   /* 极小速度验证: 约 18 rpm, 肉眼可见缓转 */

#define MAIN_PID_DEBUG_FORCE_PID      (1)
#define MAIN_PID_DEBUG_KP             (40.0f)
#define MAIN_PID_DEBUG_KI             (23.0f)
#define MAIN_PID_DEBUG_KD             (0.0f)
#endif /* MAIN_RUN_MODE_SINGLE_WHEEL */
/* ========================================================================== */
/*  ⬆⬆⬆ 单轮 PID 调试专用代码结束 ⬆⬆⬆                                            */
/* ========================================================================== */

/* 菜单渲染分频 (主循环 5ms tick * N), N=20 => 100ms 刷一次 */
#define MAIN_MENU_RENDER_DIV          (20U)

/* ========================================================================== */
/*  ⬇⬇⬇ 单轮 PID 调试辅助函数, 仅 SINGLE_WHEEL 模式下编译 ⬇⬇⬇              */
/* ========================================================================== */
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_SINGLE_WHEEL) && (1 == MAIN_PID_DEBUG_FORCE_PID)
/*
 * 把 main 内 KP/KI/KD 写进 chassis_tune_params, 仅覆盖被调试的那一个轮子,
 * 其余轮子的 PID 维持 menu/Flash 值不动. 在 chassis_ctrl_init 之后调用.
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
/* ========================================================================== */
/*  ⬆⬆⬆ 单轮 PID 调试辅助函数结束 ⬆⬆⬆                                          */
/* ========================================================================== */
main(void)
{
    uint8 menu_render_div = 0U;

    clock_init(SYSTEM_CLOCK_600M);  // 不可删除
    debug_init();                   // 调试端口初始化

    // ------------------------------------------------------------------
    // 1. 通信外设初始化
    // ------------------------------------------------------------------
    // OpenART1: 地图识别模块, 通过 UART4 上报 MAP/HEARTBEAT 帧.
#if ((MAIN_RUN_MODE == MAIN_RUN_MODE_GAME) || (MAIN_RUN_MODE == MAIN_RUN_MODE_OPENART1_TEST))
    uart_init(MAIN_OPENART1_UART, 115200, MAIN_OPENART1_UART_TX, MAIN_OPENART1_UART_RX);
    uart_rx_interrupt(MAIN_OPENART1_UART, 1);
    app_link_init();                /* P0-1: 协议解析层初始化, 必须在 uart_rx_interrupt 之后 */
#endif

#if (MAIN_RUN_MODE == MAIN_RUN_MODE_OPENART1_TEST)
    uart_init(MAIN_TEST_LOG_UART, 115200, MAIN_TEST_LOG_UART_TX, MAIN_TEST_LOG_UART_RX);
#endif

    // ------------------------------------------------------------------
    // 2. IPS200 屏幕 + 按键初始化 (调参菜单)
    // ------------------------------------------------------------------
    ips200_set_dir(IPS200_CROSSWISE);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    key_init(10);

    // ------------------------------------------------------------------
    // 3. 底盘控制子系统初始化 + 调参菜单
    //    包括: IMU 零偏标定、电机 PWM、编码器、PID 参数
    //    注意: 调用此函数前请确保车模静止放置在平面上
    // ------------------------------------------------------------------
    chassis_ctrl_init();
    chassis_menu_init();
    /* 发车位: 整数格约定 (0.5, 5.5) → 自动换算成米送入里程计原点.
     * 半整数表示发车区中心 (距左墙半格, 距上墙 5.5 格), 与实车摆放吻合. */
    chassis_ctrl_set_pose(MAIN_POS_GRID_TO_M_X(MAIN_POS_NAV_START_X_GRID),
                          MAIN_POS_GRID_TO_M_Y(MAIN_POS_NAV_START_Y_GRID),
                          0.0f);

#if (MAIN_RUN_MODE == MAIN_RUN_MODE_SINGLE_WHEEL)
    /* ⬇⬇⬇ 单轮 PID 调试启动逻辑, 姿态调试阶段这里被 #if 屏蔽 ⬇⬇⬇ */
    /* 单轮 PID 调试模式: 仅一个轮子参与闭环, 其余轮子目标恒 0;
     * 闭环执行体在 PIT_CH1 / chassis_ctrl_task_20ms 内. */
  #if (1 == MAIN_PID_DEBUG_FORCE_PID)
    main_apply_debug_wheel_pid();
  #endif
    chassis_ctrl_start_single_wheel_pid_debug((uint8)MAIN_PID_DEBUG_WHEEL_INDEX,
                                              MAIN_PID_DEBUG_TARGET_MPS);
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_YAW_HOLD)
    /* ✅ 姿态闭环调试走这里: 只设一次目标角, 后续 PIT_CH1 20ms 中断中持续闭环. */
    chassis_ctrl_hold_yaw(MAIN_HOLD_YAW_TARGET_DEG);
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_POINT_NAV)
    /* 四角遍历: 首目标由主循环在暖机完成后下发 (MAIN_POINT_NAV_WARMUP_TICKS).
     * 此处只保持 YAW_HOLD, 让 IMU KF / 编码器 LPF 先稳定,
     * 避免位置积分在系统未就绪时提前累积导致起步超调. */
    chassis_ctrl_hold_yaw(0.0f);
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_SOKO_SELFTEST)
    /* 推箱求解自测: 不驱动车体, 仅保留底层初始化和 DAP 串口输出. */
    uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);
    chassis_ctrl_stop();
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_OPENART1_TEST)
    /* OpenART1 UART4 链路测试: 只收视觉帧和打印统计, 不进入游戏状态机. */
    chassis_ctrl_stop();
    ips200_clear();
    ips200_show_string(0, 0, "OPENART1 UART4");
    ips200_show_string(0, 16, "WAIT RX...");
#else
    /* 游戏模式: 设置初始航向基准, 等待状态机调度. */
    chassis_ctrl_hold_yaw(0.0f);
#endif

    // ------------------------------------------------------------------
    // 4. PIT 定时中断初始化
    //    CH0: 5ms 姿态采样  CH1: 20ms 底盘闭环  CH2: 10ms 菜单扫描（渲染在主循环）
    // ------------------------------------------------------------------
    pit_ms_init(PIT_CH0, 5);
    pit_ms_init(PIT_CH1, 20);
    pit_ms_init(PIT_CH2, 10);

    // ------------------------------------------------------------------
    // 5. 主循环: 游戏状态机
    //    Game_Logic_Task_Run 为非阻塞函数，内部维护推箱子状态机。
    // ------------------------------------------------------------------
    // 静态调试阶段可先不运行状态机，改为手动下发网格目标点。
    // app_chassis_ctrl_move_to_grid(3, 5);

    while (1)
    {
        /*
         * P0-5: 替代 system_delay_ms(5).
         * 在此阻塞直到 PIT_CH0 5ms tick 到来, 期间 __WFI 休眠, 任意中断可唤醒.
         * 注意 wait 必须在每轮主循环工作之前调用, 保证节拍对齐 PIT 边沿.
         */
        (void)wait_for_tick();

    #if (MAIN_RUN_MODE == MAIN_RUN_MODE_SINGLE_WHEEL)
        /* ⬇⬇⬇ 单轮 PID 打印, 姿态调试阶段这里被 #if 屏蔽 ⬇⬇⬇ */
        /* 单轮 PID 调试: 100ms 打印目标速度/实际速度两列, 上位机绘曲线 */
        chassis_pid_debug_task_5ms();
    #elif (MAIN_RUN_MODE == MAIN_RUN_MODE_YAW_HOLD)
        /* ✅ 姿态闭环调试打印走这里: 50ms 打印 12 通道, 用于画角度曲线 */
        chassis_ctrl_attitude_debug_task_5ms();
    #elif (MAIN_RUN_MODE == MAIN_RUN_MODE_POINT_NAV)
        /* 路径航点遍历状态机:
         *   暖机 1s → 依次下发航点 → 全程结束后驻停于终点.
         *
         *   标志位: chassis_ctrl_is_arrived() (chassis 内部 s_arrived)
         *     == 0: 行驶中 / 旋转中 (发车时由 move_to_grid / rotate_to_deg 内部清零)
         *     == 1: 已到达当前目标
         *   到达后立即更新索引并派发下一个航点, 自动将标志位清零; 全程结束驻停. */
        {
            /* 航点动作类型 */
            #define S_NAV_MOVE_GRID  (0U)   /* chassis_ctrl_move_to_grid(x格, y格) */
            #define S_NAV_MOVE_M     (2U)   /* chassis_ctrl_move_to_m (用于半格精度) */

            /* 航点描述: { 动作, a, b }
             *   S_NAV_MOVE_GRID: a=x(格), b=y(格)
             *   S_NAV_MOVE_M:    a=x(格,运行时换算为米), b=y(格,运行时换算为米)
             *
             * 注意: 不再使用 S_NAV_ROTATE 原地旋转指令.
             * 根因: 麦轮原地旋转时滚子滑动 → 里程计 X/Y 积分偏差 → 旋转后
             *       move_to_grid 的 yaw_snap 锁住非 0° 航向 → 后续平移变成侧向
             *       strafing (精度低) → "走斜线". 去掉旋转后车辆全程锁 0° 航向,
             *       与之前可靠运行的四角遍历逻辑保持一致. */
            typedef struct { uint8 act; float a; float b; } s_nav_wp_t;
            /* ────────────────────────────────────────────────────────────
             * 当前路线总行程约 25.4m，麦轮里程计漂移 2~5% → 位置误差 50cm~1.3m.
             * 【优化建议】去掉⑥⑦⑧三个重复角落，改为5步直接回起点，
             * 可将行程缩短到 ~15m，漂移减半。若赛规要求重复访问则保留。
             * ──────────────────────────────────────────────────────────── */
            static const s_nav_wp_t s_wps[] = {
                { S_NAV_MOVE_GRID,  1.0f,  1.0f  },  /* ① 左上角 (1,1)   */
                { S_NAV_MOVE_GRID,  14.0f, 1.0f  },  /* ② 右上角 (14,1)  */
                { S_NAV_MOVE_GRID,  14.0f, 10.0f },  /* ③ 右下角 (14,10) */
                { S_NAV_MOVE_GRID,  1.0f,  10.0f },  /* ④ 左下角 (1,10)  */
                { S_NAV_MOVE_GRID,  7.0f,  5.0f  },  /* ⑤ 中心  (7,5)    */
                { S_NAV_MOVE_GRID,  1.0f,  1.0f  },  /* ⑥ 左上角 (1,1) 再次经过 */
                { S_NAV_MOVE_GRID,  14.0f, 10.0f },  /* ⑦ 右下角 (14,10) 再次经过 */
                { S_NAV_MOVE_GRID,  1.0f,  1.0f  },  /* ⑧ 左上角 (1,1)  作为回程中转 */
                { S_NAV_MOVE_M,     1.0f,  5.5f  },  /* ⑨ 回起点 (1,5.5) */
            };

            static uint8  s_wp_idx       = 0U;   /* 当前航点索引 (0 起, 到 count 停止) */
            static uint16 s_warmup_ticks = 0U;   /* 暖机计数 (5ms tick) */
            static uint8  s_nav_started  = 0U;   /* 0=暖机中, 1=遍历进行中 */

/* 航点派发辅助宏: wp 为 const s_nav_wp_t * */
#define S_NAV_DISPATCH(wp)                                                   \
    do {                                                                     \
        if (S_NAV_MOVE_GRID == (wp)->act) {                                  \
            chassis_ctrl_move_to_grid((uint8)(wp)->a, (uint8)(wp)->b);       \
        } else {                                                             \
            chassis_ctrl_move_to_m(MAIN_POS_GRID_TO_M_X((wp)->a),           \
                                   MAIN_POS_GRID_TO_M_Y((wp)->b), 0.0f);    \
        }                                                                    \
    } while (0)

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

            /* 50ms 打印: 位姿 / 目标点 / 到达标志 / 当前航点索引 */
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
                    printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d\n",
                           pose.x_m, pose.y_m,
                           tgt_x, tgt_y,
                           dist_sq,
                           pose.yaw_deg,
                           (int)chassis_ctrl_is_arrived(),
                           (int)s_wp_idx);
                }
            }
        }
    #elif (MAIN_RUN_MODE == MAIN_RUN_MODE_SOKO_SELFTEST)
        main_run_soko_selftest_periodic_5ms();
    #elif (MAIN_RUN_MODE == MAIN_RUN_MODE_OPENART1_TEST)
        {
            static uint16 s_link_print_div = 0U;
            s_link_print_div++;
            if (s_link_print_div >= 200U)
            {
                char log_buffer[128];
                int log_len;
                s_link_print_div = 0U;
                log_len = snprintf(log_buffer, sizeof(log_buffer),
                                   "LINK ok=%lu hb=%lu seq=%u crc=%lu len=%lu tout=%lu map_ms=%lu car=%u,%u\r\n",
                                   g_link_stats.frames_ok,
                                   g_link_stats.hb_cnt,
                                   g_link_stats.last_hb_seq,
                                   g_link_stats.frames_crc_err,
                                   g_link_stats.frames_len_err,
                                   g_link_stats.frames_byte_timeout,
                                   g_link_last_map_ms,
                                   g_link_car_x,
                                   g_link_car_y);
                if (log_len > 0)
                {
                    if (log_len > (int)(sizeof(log_buffer) - 1U))
                    {
                        log_len = (int)(sizeof(log_buffer) - 1U);
                    }
                    printf("%s", log_buffer);
                    uart_write_buffer(MAIN_TEST_LOG_UART, (const uint8 *)log_buffer, (uint32)log_len);
                }

                ips200_show_string(0, 16, "OK      HB      ");
                ips200_show_uint(24, 16, g_link_stats.frames_ok, 6);
                ips200_show_uint(88, 16, g_link_stats.hb_cnt, 6);
                ips200_show_string(0, 32, "CRC     LEN     ");
                ips200_show_uint(32, 32, g_link_stats.frames_crc_err, 5);
                ips200_show_uint(96, 32, g_link_stats.frames_len_err, 5);
                ips200_show_string(0, 48, "CAR     ,       ");
                ips200_show_uint(32, 48, g_link_car_x, 2);
                ips200_show_uint(56, 48, g_link_car_y, 2);
                ips200_show_string(0, 64, "MAPMS          ");
                ips200_show_uint(48, 64, g_link_last_map_ms, 8);
            }
        }
    #else
        Game_Logic_Task_Run();          /* 推箱子状态机 (非阻塞) */
    #endif

        /* 菜单渲染放到主循环，避免在 PIT 中断内刷屏造成控制节拍抖动。 */
        menu_render_div++;
        if ((MAIN_RUN_MODE != MAIN_RUN_MODE_OPENART1_TEST) &&
            (menu_render_div >= MAIN_MENU_RENDER_DIV))
        {
            menu_render_div = 0U;
            chassis_menu_render_100ms();
        }

        /* P0-5: 节拍由 wait_for_tick() 在循环顶部统一接管, 此处不再 system_delay_ms */
    }
}
