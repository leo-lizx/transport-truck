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
#include "chassis_menu.h"
#include "app_game_logic.h"
#include "app_link.h"   /* P0-1: 视觉-主控帧协议 */
#include "algo_sokoban_solver.h"
#include <math.h>       /* sqrtf — POINT_NAV 调试打印用 */
#include <stdio.h>
#include <string.h>
#include "fsl_rtwdog.h"  /* 硬件看门狗: 屏幕 SPI 卡死时 2s 自动复位 */

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
 *  顶层运行模式开关 —— 编译期八选一, 烧录时只有一种模式生效
 *  ────────────────────────────────────────────────────────────────
 *  调试递进建议:
 *    ③ 单轮 → ② 航向 → ③ 四角 → ④ 摄像头显示 → ⑤ 解算验证 → ⑧ 硬编码 → ⑦ 第一关 → ① 正式比赛
 *  ────────────────────────────────────────────────────────────────
 *  编号  宏                                 轮子?  地图来源        功能一句话
 *  ────────────────────────────────────────────────────────────────
 *   0   MAIN_RUN_MODE_GAME                  ✅    OpenART 串口    正式比赛: 摄像头识图→解算→推箱→回库
 *   1   MAIN_RUN_MODE_YAW_HOLD              ❌    无               航向保持调试: 车不动, IMU 锁死目标角度调 wz 环
 *   2   MAIN_RUN_MODE_SINGLE_WHEEL          单轮   无               单轮 PID 调试: 只让一个轮子转, 调速度环 Kp/Ki/Kd
 *   3   MAIN_RUN_MODE_POINT_NAV             ✅    无               定点导航: 按预设 9 个航点依次跑四角→回起点
 *   4   MAIN_RUN_MODE_SOKO_SELFTEST         ❌    摄像头 UART4     推箱求解自测: 接收摄像头地图帧→屏幕彩色方块显示
 *   5   MAIN_RUN_MODE_SOLVE_VERIFY          ❌    摄像头 UART4     解算验证: 收图→解算→虚拟走比赛流程, 车不动仅验算法
 *   6   MAIN_RUN_MODE_STATIC_VERIFY         ❌    代码内置          静态验证: 内置地图→解算→屏幕慢速播放, 车不动
 *   7   MAIN_RUN_MODE_LEVEL1_TEST           ✅    OpenART 串口     第一关完整流程: 收图→等人发车→推箱→回发车区
 *   8   MAIN_RUN_MODE_HARDCODED_MAP         ✅    代码内置          固定发车: 上电解算→Y轴平移发车→推箱→回库
 *   9   MAIN_RUN_MODE_OPENART2_TEST         ❌    OpenART2 UART1   分类链路自测: 只显示 BOX_CLASS, 车不动
 *   10  MAIN_RUN_MODE_LEVEL2_TEST           ✅    OpenART1+2       第二关单测: 收图→发车→分类→Stage2推箱→回库
 *=========================================================================*/
#define MAIN_RUN_MODE_GAME            (0)   /* 正式比赛: 完整视觉+推箱+底盘闭环 */
#define MAIN_RUN_MODE_YAW_HOLD        (1)   /* 航向保持: 车不动, IMU 锁角度调 yaw PID */
#define MAIN_RUN_MODE_SINGLE_WHEEL    (2)   /* 单轮调试: 一个轮子转, 调速度环参数 */
#define MAIN_RUN_MODE_POINT_NAV       (3)   /* 定点导航: 预设航点四角遍历, 测里程计精度 */
#define MAIN_RUN_MODE_SOKO_SELFTEST   (4)   /* 推箱自测: 收摄像头地图→屏幕色块, 车不动 */
#define MAIN_RUN_MODE_SOLVE_VERIFY    (5)   /* 解算验证: 收图→解算→虚拟跑流程显示, 车不动 */
#define MAIN_RUN_MODE_STATIC_VERIFY   (6)   /* 静态验证: 内置地图→解算→屏幕慢速播放, 车不动 */
#define MAIN_RUN_MODE_LEVEL1_TEST     (7)   /* 第一关测试: 收图→等发车→推箱→回库 (最接近比赛) */
#define MAIN_RUN_MODE_HARDCODED_MAP   (8)   /* 硬编码地图: 上电解算→Y轴平移发车→跑→回发车点 */
#define MAIN_RUN_MODE_OPENART2_TEST   (9)   /* OpenART2 分类链路测试: 屏幕显示 BOX/TARGET/NONE, 车不动 */
#define MAIN_RUN_MODE_LEVEL2_TEST     (10)  /* 第二关测试: 收图→固定发车→分类识别→Stage2推箱→回库 */

/* ═══════════ 改下面这行切换运行模式 (0~10) ═══════════ */
#define MAIN_RUN_MODE                 (MAIN_RUN_MODE_HARDCODED_MAP)  /* 只接收摄像头地图并显示，底盘保持停止 */
/* ═══════════ 改上面这行切换运行模式 (0~10) ═══════════ */

/* OpenART1 地图链路硬件口: 若实测 UART4 走 D0/D1, 只改下面两行宏. */
#define MAIN_OPENART1_UART            (UART_4)
#define MAIN_OPENART1_UART_TX         (UART4_TX_C16)
#define MAIN_OPENART1_UART_RX         (UART4_RX_C17)

/* OpenART2 分类链路硬件口: UART1 接分类摄像头 BOX_CLASS/HEARTBEAT 帧. */
#define MAIN_OPENART2_UART            (UART_1)
#define MAIN_OPENART2_UART_TX         (UART1_TX_B12)
#define MAIN_OPENART2_UART_RX         (UART1_RX_B13)

/* ========================================================================== */
/* 位置/起点宏（上提至所有调试模式之前, 避免 LEVEL1_TEST/HARDCODED_MAP 等
 * 模式内引用时未定义）.
 *
 * 模式8实车从 (1,5.5) 上电, 暖机后先沿 Y 轴固定平移到 (1,5),
 * 再从 (1,5) 对应的整数格开始按 Stage1 航点跑完整关. */
/* ========================================================================== */
#define MAIN_POS_NAV_START_X_GRID     (1.0f)
#define MAIN_POS_NAV_START_Y_GRID     (5.5f)

#define MAIN_POS_HCM_HOME_X_GRID      (MAIN_POS_NAV_START_X_GRID)
#define MAIN_POS_HCM_HOME_Y_GRID      (MAIN_POS_NAV_START_Y_GRID)

#define MAIN_POS_HCM_LAUNCH_TARGET_X_GRID  (MAIN_POS_HCM_HOME_X_GRID)
#define MAIN_POS_HCM_LAUNCH_TARGET_Y_GRID  (5.0f)
#define MAIN_POS_HCM_SOLVE_START_X_GRID    (1)
#define MAIN_POS_HCM_SOLVE_START_Y_GRID    (5)

#define MAIN_POS_GRID_TO_M_X(g)       (((float)(g) - 0.5f) * CHASSIS_GRID_STEP_X_M)
#define MAIN_POS_GRID_TO_M_Y(g)       (((float)(g) - 0.5f) * CHASSIS_GRID_STEP_Y_M)
#define MAIN_POINT_NAV_WARMUP_TICKS   (200U)   /* 200 * 5ms = 1s warmup */
#define MAIN_LEVEL_TEST_NAV_TIMEOUT_TICKS (6000U) /* 30s per dispatched target */

#if (MAIN_RUN_MODE == MAIN_RUN_MODE_LEVEL1_TEST) || (MAIN_RUN_MODE == MAIN_RUN_MODE_HARDCODED_MAP)
/* s_soko_selftest_map + main_selftest_char_to_map: 供 HARDCODED_MAP (模式8) 使用.
 * SOKO_SELFTEST (模式4) 已改为从摄像头 UART4 接收地图, 不再引用内置地图. */
/*
 * 固定自测图: 由用户编辑的竖版地图整理为 12x16 横版格式.
 * 目的: 保留原始关卡结构, 同时满足求解器固定 12 行 16 列的输入约束.
 *
 * 注: 地图本身不带发车位置标记 (无 '@'), 发车格写死为
 *     (CHASSIS_START_GRID_X, CHASSIS_START_GRID_Y) = (1, 6),
 *     与 chassis_config.h / 底层 init 完全一致.
 */
static const char s_soko_selftest_map[MAP_ROWS][MAP_COLS + 1] = {
"################",
"#-----.--------#",
"#-#--#--------.#",
"#-$--#-#---###-#",
"#--$##-----.$--#",
"#----------.-#-#",
"#@---##-#-----$#",
"#---##-$--#--#-#",
"#--------#---#-#",
"#------.#-#-#--#",
"#----$------.--#",
"################"
};
/* 旧地图:
"################",
"#--------------#",
"#-.--#---------#",
"#--------------#",
"#---#------$---#",
"#---#--$----$--#",
"#@----$--##--.-#",
"#--#------#--#-#",
"#----$-.#------#",
"#------.#-#----#",
"#------------.-#",
"################"
*/

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

/*
 * SOKO_SELFTEST 模式下不再需要主循环 5ms 钩子:
 *   OpenART 摄像头通过 UART4 持续发送 MAP/HEARTBEAT 帧 → app_link ISR 解析 →
 *   每帧合法 MAP 落地时自动刷新 g_link_last_map_ms → chassis_menu_render_100ms()
 *   检测到地图新鲜(≤500ms)即在 IPS200 上绘制彩色方块.
 *   车不驱动, 无串口 printf.
 */
#endif

/* ============================================================================
 *  LEVEL1_TEST 模式: 第一关完整测试
 *  流程:
 *    1. WAIT_MAP     — 等待 OpenART 通过 UART4 发来地图帧
 *    2. SOLVE        — 冻结该地图后解算推箱子路径 (Stage1)
 *    3. WARMUP       — 解算成功后暖机 1s (IMU/编码器稳定)
 *    4. LAUNCH_FIXED — 与模式8一致, 从固定发车点平移到解算起点
 *    5. PUSH_BOXES   — 逐子箱跑航点 (不使用视觉矫正, 纯里程计)
 *    6. RETURN_HOME  — 推完后回固定发车点
 *    7. DONE         — 驻停
 *
 *  关键约束:
 *    - 如果没有收到地图, 车不运动 (停在发车区)
 *    - 不使用视觉矫正, 纯里程计导航
 *    - 完成后回到发车区
 * ============================================================================ */
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_LEVEL1_TEST)

typedef enum {
    L1_PHASE_WAIT_MAP = 0,   /* 等待 OpenART 地图帧 */
    L1_PHASE_SOLVE,          /* 解算 (瞬时, 同一 tick 内完成) */
    L1_PHASE_WARMUP,         /* 暖机等待 IMU/编码器稳定 */
    L1_PHASE_LAUNCH_FIXED,   /* same as mode 8: fixed launch to solve start */
    L1_PHASE_PUSH_BOXES,     /* 逐航点推箱 */
    L1_PHASE_RETURN_HOME,    /* 回发车区起点 */
    L1_PHASE_DONE,           /* 完成驻停 */
    L1_PHASE_FAIL            /* 首张地图或导航失败，锁定驻停 */
} l1_phase_e;

static SokoFullSolution_t  s_l1_solution;
static SokoWaypointPath_t  s_l1_waypoints;
static uint8               s_l1_map[MAP_ROWS][MAP_COLS];
static l1_phase_e          s_l1_phase        = L1_PHASE_WAIT_MAP;
static uint8               s_l1_solve_ok     = 0U;
static uint8               s_l1_sub_idx      = 0U;
static uint16              s_l1_wp_idx       = 0U;
static uint8               s_l1_navigating   = 0U;
static uint16              s_l1_warmup_ticks = 0U;
static uint16              s_l1_nav_ticks    = 0U;
static uint32              s_l1_map_recv_ms  = 0U;

/*
 * 解算: 拿到地图快照后调 Stage1 求解
 */
static void main_l1_solve(void)
{
    Point_t start_pos;
    uint8 r;

    start_pos.x = (int8)MAIN_POS_HCM_SOLVE_START_X_GRID;
    start_pos.y = (int8)MAIN_POS_HCM_SOLVE_START_Y_GRID;

    printf("L1_MAP_BEGIN\n");
    for (r = 0U; r < MAP_ROWS; r++)
    {
        char line[MAP_COLS + 1];
        uint8 c;
        for (c = 0U; c < MAP_COLS; c++)
        {
            switch (s_l1_map[r][c])
            {
            case MAP_WALL:   line[c] = '#'; break;
            case MAP_TARGET: line[c] = '.'; break;
            case MAP_BOX:    line[c] = '$'; break;
            case MAP_BOMB:   line[c] = '*'; break;
            default:         line[c] = '-'; break;
            }
        }
        line[MAP_COLS] = '\0';
        printf("%s\n", line);
    }
    printf("L1_MAP_END\n");
    printf("L1_PLAYER_START=%.2f,%.2f (grid=%d,%d)\n",
           (double)MAIN_POS_HCM_LAUNCH_TARGET_X_GRID,
           (double)MAIN_POS_HCM_LAUNCH_TARGET_Y_GRID,
           (int)start_pos.x, (int)start_pos.y);

    for (r = 0U; r < MAP_ROWS; ++r)
    {
        uint8 c;
        for (c = 0U; c < MAP_COLS; ++c)
        {
            if (s_l1_map[r][c] == MAP_BOMB)
            {
                printf("L1_ERR=NOT_LEVEL1_BOMB_PRESENT\n");
                s_l1_solve_ok = 0U;
                return;
            }
        }
    }

    memset(&s_l1_solution, 0, sizeof(s_l1_solution));
    if (!Sokoban_Solve_Stage1(s_l1_map, start_pos, &s_l1_solution))
    {
        printf("L1_ERR=NO_SOLUTION\n");
        s_l1_solve_ok = 0U;
        return;
    }
    if (!s_l1_solution.is_solved || s_l1_solution.total_boxes == 0U)
    {
        printf("L1_ERR=UNSOLVED boxes=%d solved=%d\n",
               (int)s_l1_solution.total_boxes,
               (int)s_l1_solution.is_solved);
        s_l1_solve_ok = 0U;
        return;
    }

    /* 装载首子箱航点 */
    s_l1_sub_idx = 0U;
    Sokoban_Actions_To_Waypoints(s_l1_solution.sub_solutions[0].actions,
                                 s_l1_solution.sub_solutions[0].count,
                                 start_pos,
                                 &s_l1_waypoints);
    s_l1_wp_idx = 0U;
    s_l1_navigating = 0U;
    s_l1_solve_ok = 1U;

    printf("L1_SOLVED=1 boxes=%d wp0=%d\n",
           (int)s_l1_solution.total_boxes,
           (int)s_l1_waypoints.count);
}

/*
 * WAIT_MAP 阶段: 等待 OpenART 发来地图
 */
static void main_l1_wait_map_5ms(void)
{
    static uint16 s_div = 0U;

    uint32 now_recv_ms = g_link_last_map_ms;
    if (now_recv_ms != 0U && now_recv_ms != s_l1_map_recv_ms)
    {
        s_l1_map_recv_ms = now_recv_ms;
        app_link_get_map_snapshot(s_l1_map);
        printf("L1_MAP_RECV ms=%lu ok=%lu crc=%lu len=%lu\n",
               (unsigned long)now_recv_ms,
               (unsigned long)g_link_stats.frames_ok,
               (unsigned long)g_link_stats.frames_crc_err,
               (unsigned long)g_link_stats.frames_len_err);

        /* Freeze this OpenART1 map before solving. Later frames cannot change this run. */
        uart_rx_interrupt(MAIN_OPENART1_UART, 0);
        app_link_inject_static_map(s_l1_map);
        s_l1_map_recv_ms = g_link_last_map_ms;
        printf("L1_MAP_LOCKED\n");
        s_l1_phase = L1_PHASE_SOLVE;
        return;
    }

    /* 还没收到, 1s 打一次状态 */
    if (++s_div >= 200U)
    {
        s_div = 0U;
        printf("L1_WAIT_MAP hb=%lu ok=%lu crc=%lu\n",
               (unsigned long)g_link_stats.hb_cnt,
               (unsigned long)g_link_stats.frames_ok,
               (unsigned long)g_link_stats.frames_crc_err);
    }
}

/*
 * 主循环 5ms tick 总入口
 */
static void main_l1_keep_locked_map_fresh(void)
{
    if ((s_l1_solve_ok != 0U) && (s_l1_phase >= L1_PHASE_WARMUP))
    {
        g_link_last_map_ms = app_link_get_ms();
    }
}

static uint8 main_l1_nav_watchdog_5ms(void)
{
    if (s_l1_navigating == 0U)
    {
        s_l1_nav_ticks = 0U;
        return 0U;
    }
    if (++s_l1_nav_ticks <= MAIN_LEVEL_TEST_NAV_TIMEOUT_TICKS)
    {
        return 0U;
    }

    chassis_ctrl_stop();
    s_l1_navigating = 0U;
    s_l1_phase = L1_PHASE_FAIL;
    printf("L1_ERR=NAV_TIMEOUT\n");
    return 1U;
}

static void main_run_level1_test_5ms(void)
{
    main_l1_keep_locked_map_fresh();
    if (main_l1_nav_watchdog_5ms() != 0U)
    {
        return;
    }

    switch (s_l1_phase)
    {
    case L1_PHASE_WAIT_MAP:
        main_l1_wait_map_5ms();
        return;

    case L1_PHASE_SOLVE:
        main_l1_solve();
        if (s_l1_solve_ok)
        {
            s_l1_phase = L1_PHASE_WARMUP;
            s_l1_warmup_ticks = 0U;
            printf("L1_WARMUP start\n");
        }
        else
        {
            /* 单关测试只消费上电后的首张完整地图；失败后保持锁定，避免误测下一关。 */
            chassis_ctrl_stop();
            s_l1_phase = L1_PHASE_FAIL;
            printf("L1_FAIL first MAP remains locked\n");
        }
        return;

    case L1_PHASE_WARMUP:
        /* 暖机: 等 IMU/编码器稳定 */
        if (++s_l1_warmup_ticks >= MAIN_POINT_NAV_WARMUP_TICKS)
        {
            printf("L1_WARMUP_DONE launch along y to solve start\n");
            s_l1_phase = L1_PHASE_LAUNCH_FIXED;
            chassis_ctrl_move_to_m(
                MAIN_POS_GRID_TO_M_X(MAIN_POS_HCM_LAUNCH_TARGET_X_GRID),
                MAIN_POS_GRID_TO_M_Y(MAIN_POS_HCM_LAUNCH_TARGET_Y_GRID),
                0.0f);
            s_l1_navigating = 1U;
        }
        return;

    case L1_PHASE_LAUNCH_FIXED:
        if (chassis_ctrl_is_arrived())
        {
            printf("L1_LAUNCH_DONE start pushing\n");
            s_l1_navigating = 0U;
            s_l1_phase = L1_PHASE_PUSH_BOXES;
        }
        return;

    case L1_PHASE_PUSH_BOXES:
        if (!s_l1_navigating)
        {
            /* 检查当前子箱航点是否已跑完 */
            if (s_l1_wp_idx >= s_l1_waypoints.count)
            {
                /* 当前子箱完成, 切下一子箱 */
                s_l1_sub_idx++;
                if (s_l1_sub_idx >= s_l1_solution.total_boxes)
                {
                    /* 全部箱子推完 → 回发车区 */
                    printf("L1_ALL_BOXES_DONE, returning home\n");
                    s_l1_phase = L1_PHASE_RETURN_HOME;
                    chassis_ctrl_move_to_m(
                        MAIN_POS_GRID_TO_M_X(MAIN_POS_HCM_HOME_X_GRID),
                        MAIN_POS_GRID_TO_M_Y(MAIN_POS_HCM_HOME_Y_GRID),
                        0.0f);
                    s_l1_navigating = 1U;
                    return;
                }
                /* 装载下一子箱航点 */
                Sokoban_Actions_To_Waypoints(
                    s_l1_solution.sub_solutions[s_l1_sub_idx].actions,
                    s_l1_solution.sub_solutions[s_l1_sub_idx].count,
                    s_l1_solution.player_end_pos[s_l1_sub_idx - 1U],
                    &s_l1_waypoints);
                s_l1_wp_idx = 0U;
                printf("L1_NEXT_BOX sub=%d wp=%d\n",
                       (int)s_l1_sub_idx, (int)s_l1_waypoints.count);
            }
            /* 派发当前航点 */
            {
                Point_t next = s_l1_waypoints.points[s_l1_wp_idx];
                chassis_ctrl_move_to_grid((uint8)next.x, (uint8)next.y);
                s_l1_navigating = 1U;
            }
            return;
        }

        /* 已派发, 等到位 */
        if (!chassis_ctrl_is_arrived())
        {
            return;
        }
        s_l1_navigating = 0U;
        s_l1_wp_idx++;
        return;

    case L1_PHASE_RETURN_HOME:
        if (chassis_ctrl_is_arrived())
        {
            chassis_ctrl_stop();
            printf("L1_DONE returned to launch zone\n");
            s_l1_phase = L1_PHASE_DONE;
        }
        return;

    case L1_PHASE_DONE:
    case L1_PHASE_FAIL:
        /* 单关成功或失败后均永久驻停，只有复位才允许接收下一张地图。 */
        chassis_ctrl_stop();
        return;

    default:
        return;
    }
}

/* 50ms 调试打印 */
static void main_run_level1_test_log_50ms(void)
{
    static uint8 s_div = 0U;
    if (++s_div < 10U) return;
    s_div = 0U;
    {
        chassis_pose_t pose = chassis_ctrl_get_pose();
        float tgt_x = 0.0f, tgt_y = 0.0f;
        chassis_ctrl_get_point_nav_target_m(&tgt_x, &tgt_y);
        printf("L1: phase=%d sub=%d/%d wp=%d/%d pos=%.3f,%.3f tgt=%.3f,%.3f arr=%d\n",
               (int)s_l1_phase,
               (int)s_l1_sub_idx, (int)(s_l1_solve_ok ? s_l1_solution.total_boxes : 0),
               (int)s_l1_wp_idx, (int)s_l1_waypoints.count,
               pose.x_m, pose.y_m,
               tgt_x, tgt_y,
               (int)chassis_ctrl_is_arrived());
    }
}

#endif /* MAIN_RUN_MODE_LEVEL1_TEST */

/* ============================================================================
 * LEVEL2_TEST mode (10): standalone Stage2 flow.
 *   WAIT_MAP -> WARMUP -> fixed launch -> classify tour -> Stage2 solve
 *   -> push boxes -> return home -> DONE.
 * ============================================================================ */
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_LEVEL2_TEST)

typedef enum {
    L2_PHASE_WAIT_MAP = 0,
    L2_PHASE_WARMUP,
    L2_PHASE_LAUNCH_FIXED,
    L2_PHASE_RECOGNIZE,
    L2_PHASE_SOLVE,
    L2_PHASE_PUSH_BOXES,
    L2_PHASE_RETURN_HOME,
    L2_PHASE_DONE,
    L2_PHASE_FAIL
} l2_phase_e;

static SokoFullSolution_t  s_l2_solution;
static SokoWaypointPath_t  s_l2_waypoints;
static uint8               s_l2_map[MAP_ROWS][MAP_COLS];
static uint8               s_l2_box_to_target[SOKOBAN_MAX_BOXES];
static l2_phase_e          s_l2_phase        = L2_PHASE_WAIT_MAP;
static uint8               s_l2_solve_ok     = 0U;
static uint8               s_l2_sub_idx      = 0U;
static uint16              s_l2_wp_idx       = 0U;
static uint8               s_l2_navigating   = 0U;
static uint8               s_l2_recog_started = 0U;
static uint16              s_l2_warmup_ticks = 0U;
static uint16              s_l2_nav_ticks    = 0U;
static uint32              s_l2_map_recv_ms  = 0U;

static Point_t main_l2_current_grid(void)
{
    chassis_pose_t pose = chassis_ctrl_get_pose();
    Point_t p;
    p.x = (int8)chassis_m_to_grid_x(pose.x_m);
    p.y = (int8)chassis_m_to_grid_y(pose.y_m);
    return p;
}

static uint8 main_l2_map_has_bomb(void)
{
    uint8 r, c;
    for (r = 0U; r < MAP_ROWS; ++r)
    {
        for (c = 0U; c < MAP_COLS; ++c)
        {
            if (s_l2_map[r][c] == MAP_BOMB)
            {
                return 1U;
            }
        }
    }
    return 0U;
}

static uint8 main_l2_box_count(void)
{
    uint8 r, c, count = 0U;
    for (r = 0U; r < MAP_ROWS; ++r)
    {
        for (c = 0U; c < MAP_COLS; ++c)
        {
            if (s_l2_map[r][c] == MAP_BOX)
            {
                ++count;
            }
        }
    }
    return count;
}

static void main_l2_print_map(void)
{
    uint8 r, c;
    printf("L2_MAP_BEGIN\n");
    for (r = 0U; r < MAP_ROWS; ++r)
    {
        char line[MAP_COLS + 1];
        for (c = 0U; c < MAP_COLS; ++c)
        {
            switch (s_l2_map[r][c])
            {
            case MAP_WALL:   line[c] = '#'; break;
            case MAP_TARGET: line[c] = '.'; break;
            case MAP_BOX:    line[c] = '$'; break;
            case MAP_BOMB:   line[c] = '*'; break;
            default:         line[c] = '-'; break;
            }
        }
        line[MAP_COLS] = '\0';
        printf("%s\n", line);
    }
    printf("L2_MAP_END\n");
}

static void main_l2_wait_map_5ms(void)
{
    static uint16 s_div = 0U;
    uint32 now_recv_ms = g_link_last_map_ms;

    if (now_recv_ms != 0U && now_recv_ms != s_l2_map_recv_ms)
    {
        s_l2_map_recv_ms = now_recv_ms;
        app_link_get_map_snapshot(s_l2_map);
        printf("L2_MAP_RECV ms=%lu ok=%lu crc=%lu len=%lu\n",
               (unsigned long)now_recv_ms,
               (unsigned long)g_link_stats.frames_ok,
               (unsigned long)g_link_stats.frames_crc_err,
               (unsigned long)g_link_stats.frames_len_err);
        main_l2_print_map();

        uart_rx_interrupt(MAIN_OPENART1_UART, 0);
        app_link_inject_static_map(s_l2_map);
        s_l2_map_recv_ms = g_link_last_map_ms;
        if (main_l2_map_has_bomb() != 0U)
        {
            chassis_ctrl_stop();
            s_l2_phase = L2_PHASE_FAIL;
            printf("L2_ERR=NOT_LEVEL2_BOMB_PRESENT first MAP remains locked\n");
            return;
        }
        memset(s_l2_box_to_target, 0, sizeof(s_l2_box_to_target));
        App_Recognize_Reset();
        s_l2_recog_started = 0U;
        s_l2_phase = L2_PHASE_WARMUP;
        s_l2_warmup_ticks = 0U;
        printf("L2_MAP_LOCKED warmup\n");
        return;
    }

    if (++s_div >= 200U)
    {
        s_div = 0U;
        printf("L2_WAIT_MAP hb=%lu ok=%lu crc=%lu class_hb=%lu class_ok=%lu\n",
               (unsigned long)g_link_stats.hb_cnt,
               (unsigned long)g_link_stats.frames_ok,
               (unsigned long)g_link_stats.frames_crc_err,
               (unsigned long)g_link_class_stats.hb_cnt,
               (unsigned long)g_link_class_stats.frames_ok);
    }
}

static void main_l2_recognize_5ms(void)
{
    AppRecognizeStatus_e r;
    Point_t cur = main_l2_current_grid();

    if (s_l2_recog_started == 0U)
    {
        memset(s_l2_box_to_target, 0, sizeof(s_l2_box_to_target));
        App_Recognize_Reset();
        s_l2_recog_started = 1U;
        printf("L2_RECOG_START grid=%d,%d boxes=%d\n",
               (int)cur.x, (int)cur.y, (int)main_l2_box_count());
    }

    r = App_Recognize_Tick(s_l2_map, cur, main_l2_map_has_bomb(),
                           2U, s_l2_box_to_target);
    if ((r == APP_RECOG_DONE_OK) || (r == APP_RECOG_DONE_NO_NEED))
    {
        uint8 i, box_count = main_l2_box_count();
        printf("L2_RECOG_DONE map:");
        for (i = 0U; i < box_count && i < (uint8)SOKOBAN_MAX_BOXES; ++i)
        {
            printf(" %d->%d", (int)i, (int)s_l2_box_to_target[i]);
        }
        printf("\n");
        s_l2_phase = L2_PHASE_SOLVE;
        return;
    }
    if (r == APP_RECOG_FAIL)
    {
        AppRecognizeDebug_t dbg;
        App_Recognize_Get_Debug(&dbg);
        chassis_ctrl_stop();
        printf("L2_RECOG_FAIL sub=%d cur=%d visited=%d rb=%d rt=%d sample=%d\n",
               (int)dbg.sub_state,
               (int)dbg.current_idx,
               (int)dbg.visited_count,
               (int)dbg.resolved_box,
               (int)dbg.resolved_target,
               (int)dbg.sample_count);
        s_l2_phase = L2_PHASE_FAIL;
    }
}

static void main_l2_solve(void)
{
    Point_t start_pos = main_l2_current_grid();
    uint8 box_count = main_l2_box_count();

    printf("L2_SOLVE_START grid=%d,%d boxes=%d\n",
           (int)start_pos.x, (int)start_pos.y, (int)box_count);

    memset(&s_l2_solution, 0, sizeof(s_l2_solution));
    if ((box_count == 0U) ||
        !Sokoban_Solve_Stage2(s_l2_map, start_pos, s_l2_box_to_target,
                              box_count, &s_l2_solution) ||
        (s_l2_solution.is_solved == 0U) ||
        (s_l2_solution.total_boxes == 0U))
    {
        printf("L2_ERR=NO_STAGE2_SOLUTION solved=%d boxes=%d\n",
               (int)s_l2_solution.is_solved,
               (int)s_l2_solution.total_boxes);
        s_l2_solve_ok = 0U;
        return;
    }

    s_l2_sub_idx = 0U;
    Sokoban_Actions_To_Waypoints(s_l2_solution.sub_solutions[0].actions,
                                 s_l2_solution.sub_solutions[0].count,
                                 start_pos,
                                 &s_l2_waypoints);
    s_l2_wp_idx = 0U;
    s_l2_navigating = 0U;
    s_l2_solve_ok = 1U;

    printf("L2_SOLVED=1 boxes=%d wp0=%d\n",
           (int)s_l2_solution.total_boxes,
           (int)s_l2_waypoints.count);
}

static void main_l2_keep_locked_map_fresh(void)
{
    if (s_l2_phase != L2_PHASE_WAIT_MAP)
    {
        g_link_last_map_ms = app_link_get_ms();
    }
}

static uint8 main_l2_nav_watchdog_5ms(void)
{
    if (s_l2_navigating == 0U)
    {
        s_l2_nav_ticks = 0U;
        return 0U;
    }
    if (++s_l2_nav_ticks <= MAIN_LEVEL_TEST_NAV_TIMEOUT_TICKS)
    {
        return 0U;
    }

    chassis_ctrl_stop();
    s_l2_navigating = 0U;
    s_l2_phase = L2_PHASE_FAIL;
    printf("L2_ERR=NAV_TIMEOUT\n");
    return 1U;
}

static void main_run_level2_test_5ms(void)
{
    main_l2_keep_locked_map_fresh();
    if (main_l2_nav_watchdog_5ms() != 0U)
    {
        return;
    }

    switch (s_l2_phase)
    {
    case L2_PHASE_WAIT_MAP:
        main_l2_wait_map_5ms();
        return;

    case L2_PHASE_WARMUP:
        if (++s_l2_warmup_ticks >= MAIN_POINT_NAV_WARMUP_TICKS)
        {
            printf("L2_WARMUP_DONE launch along y to solve start\n");
            s_l2_phase = L2_PHASE_LAUNCH_FIXED;
            chassis_ctrl_move_to_m(
                MAIN_POS_GRID_TO_M_X(MAIN_POS_HCM_LAUNCH_TARGET_X_GRID),
                MAIN_POS_GRID_TO_M_Y(MAIN_POS_HCM_LAUNCH_TARGET_Y_GRID),
                0.0f);
            s_l2_navigating = 1U;
        }
        return;

    case L2_PHASE_LAUNCH_FIXED:
        if (chassis_ctrl_is_arrived())
        {
            printf("L2_LAUNCH_DONE start classify tour\n");
            s_l2_navigating = 0U;
            s_l2_recog_started = 0U;
            s_l2_phase = L2_PHASE_RECOGNIZE;
        }
        return;

    case L2_PHASE_RECOGNIZE:
        main_l2_recognize_5ms();
        return;

    case L2_PHASE_SOLVE:
        main_l2_solve();
        if (s_l2_solve_ok)
        {
            s_l2_phase = L2_PHASE_PUSH_BOXES;
            printf("L2_PUSH_START\n");
        }
        else
        {
            chassis_ctrl_stop();
            s_l2_phase = L2_PHASE_FAIL;
        }
        return;

    case L2_PHASE_PUSH_BOXES:
        if (!s_l2_navigating)
        {
            if (s_l2_wp_idx >= s_l2_waypoints.count)
            {
                s_l2_sub_idx++;
                if (s_l2_sub_idx >= s_l2_solution.total_boxes)
                {
                    printf("L2_ALL_BOXES_DONE, returning home\n");
                    s_l2_phase = L2_PHASE_RETURN_HOME;
                    chassis_ctrl_move_to_m(
                        MAIN_POS_GRID_TO_M_X(MAIN_POS_HCM_HOME_X_GRID),
                        MAIN_POS_GRID_TO_M_Y(MAIN_POS_HCM_HOME_Y_GRID),
                        0.0f);
                    s_l2_navigating = 1U;
                    return;
                }

                Sokoban_Actions_To_Waypoints(
                    s_l2_solution.sub_solutions[s_l2_sub_idx].actions,
                    s_l2_solution.sub_solutions[s_l2_sub_idx].count,
                    s_l2_solution.player_end_pos[s_l2_sub_idx - 1U],
                    &s_l2_waypoints);
                s_l2_wp_idx = 0U;
                printf("L2_NEXT_BOX sub=%d wp=%d\n",
                       (int)s_l2_sub_idx, (int)s_l2_waypoints.count);
            }
            {
                Point_t next = s_l2_waypoints.points[s_l2_wp_idx];
                chassis_ctrl_move_to_grid((uint8)next.x, (uint8)next.y);
                s_l2_navigating = 1U;
            }
            return;
        }

        if (!chassis_ctrl_is_arrived())
        {
            return;
        }
        s_l2_navigating = 0U;
        s_l2_wp_idx++;
        return;

    case L2_PHASE_RETURN_HOME:
        if (chassis_ctrl_is_arrived())
        {
            chassis_ctrl_stop();
            printf("L2_DONE returned to launch zone\n");
            s_l2_phase = L2_PHASE_DONE;
        }
        return;

    case L2_PHASE_DONE:
    case L2_PHASE_FAIL:
    default:
        chassis_ctrl_stop();
        return;
    }
}

static void main_run_level2_test_log_50ms(void)
{
    static uint8 s_div = 0U;
    if (++s_div < 10U) return;
    s_div = 0U;
    {
        chassis_pose_t pose = chassis_ctrl_get_pose();
        float tgt_x = 0.0f, tgt_y = 0.0f;
        chassis_ctrl_get_point_nav_target_m(&tgt_x, &tgt_y);
        printf("L2: phase=%d sub=%d/%d wp=%d/%d pos=%.3f,%.3f tgt=%.3f,%.3f arr=%d\n",
               (int)s_l2_phase,
               (int)s_l2_sub_idx, (int)(s_l2_solve_ok ? s_l2_solution.total_boxes : 0),
               (int)s_l2_wp_idx, (int)s_l2_waypoints.count,
               pose.x_m, pose.y_m,
               tgt_x, tgt_y,
               (int)chassis_ctrl_is_arrived());

        if (s_l2_phase == L2_PHASE_RECOGNIZE)
        {
            AppRecognizeDebug_t dbg;
            App_Recognize_Get_Debug(&dbg);
            printf("L2_RECOG: sub=%d cur=%d/%d kind=%d cls=%d sample=%d visited=%d rb=%d rt=%d\n",
                   (int)dbg.sub_state,
                   (int)dbg.current_idx,
                   (int)dbg.total_targets,
                   (int)dbg.current_kind,
                   (int)dbg.current_class_id,
                   (int)dbg.sample_count,
                   (int)dbg.visited_count,
                   (int)dbg.resolved_box,
                   (int)dbg.resolved_target);
        }
    }
}

#endif /* MAIN_RUN_MODE_LEVEL2_TEST */

/* ============================================================================
 *  HARDCODED_MAP 模式: 硬编码地图 → 上电直接解算 → 暖机 → Y轴平移发车 → 跑航点 → 回发车区
 *  流程:
 *    1. SOLVE        — 上电后立即用编译期字面量地图解算推箱子路径
 *    2. WARMUP       — 解算成功后暖机 1s (IMU/编码器稳定)
 *    3. LAUNCH       — 从固定发车点沿 Y 轴平移到解算起点
 *    4. PUSH_BOXES   — 逐子箱跑航点 (纯里程计导航)
 *    5. RETURN_HOME  — 推完后回固定发车点
 *    6. DONE         — 驻停
 *
 *  使用方法:
 *    - 直接修改 s_soko_selftest_map 中的地图内容
 *    - 编译烧录后上电即自动解算并开始运行
 *    - 无需等待串口地图帧, 无需人工发车
 * ============================================================================ */
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_HARDCODED_MAP)

typedef enum {
    HCM_PHASE_SOLVE = 0,     /* 上电立即解算 */
    HCM_PHASE_WARMUP,        /* 暖机等待 IMU/编码器稳定 */
    HCM_PHASE_LAUNCH_FIXED,  /* 固定发车点沿 Y 轴平移到解算起点 */
    HCM_PHASE_PUSH_BOXES,    /* 逐航点推箱 */
    HCM_PHASE_RETURN_HOME,   /* 回固定发车点 */
    HCM_PHASE_DONE           /* 完成驻停 */
} hcm_phase_e;

static SokoFullSolution_t  s_hcm_solution;
static SokoWaypointPath_t  s_hcm_waypoints;
static uint8               s_hcm_map[MAP_ROWS][MAP_COLS];
static hcm_phase_e         s_hcm_phase        = HCM_PHASE_SOLVE;
static uint8               s_hcm_solve_ok     = 0U;
static uint8               s_hcm_sub_idx      = 0U;
static uint16              s_hcm_wp_idx       = 0U;
static uint8               s_hcm_navigating   = 0U;
static uint16              s_hcm_warmup_ticks = 0U;

/*
 * 上电解算: 把编译期字面量地图转为 uint8 数组, 调 Stage1 求解
 */
static void main_hcm_solve(void)
{
    Point_t start_pos;
    uint8 r, c;

    start_pos.x = (int8)MAIN_POS_HCM_SOLVE_START_X_GRID;
    start_pos.y = (int8)MAIN_POS_HCM_SOLVE_START_Y_GRID;

    /* 字面量地图 → uint8 数组 */
    for (r = 0U; r < MAP_ROWS; r++)
    {
        for (c = 0U; c < MAP_COLS; c++)
        {
            s_hcm_map[r][c] = main_selftest_char_to_map(s_soko_selftest_map[r][c]);
        }
    }

    /* 打印地图供调试 */
    printf("HCM_MAP_BEGIN\n");
    for (r = 0U; r < MAP_ROWS; r++)
    {
        printf("%s\n", s_soko_selftest_map[r]);
    }
    printf("HCM_MAP_END\n");
    printf("HCM_PLAYER_START=%.2f,%.2f (grid=%d,%d)\n",
           (double)MAIN_POS_HCM_LAUNCH_TARGET_X_GRID,
           (double)MAIN_POS_HCM_LAUNCH_TARGET_Y_GRID,
           (int)start_pos.x, (int)start_pos.y);

    /* 调用求解器 */
    memset(&s_hcm_solution, 0, sizeof(s_hcm_solution));
    if (!Sokoban_Solve_Stage1(s_hcm_map, start_pos, &s_hcm_solution))
    {
        printf("HCM_ERR=NO_SOLUTION\n");
        s_hcm_solve_ok = 0U;
        return;
    }
    if (!s_hcm_solution.is_solved || s_hcm_solution.total_boxes == 0U)
    {
        printf("HCM_ERR=UNSOLVED boxes=%d solved=%d\n",
               (int)s_hcm_solution.total_boxes,
               (int)s_hcm_solution.is_solved);
        s_hcm_solve_ok = 0U;
        return;
    }

    /* 装载首子箱航点 */
    s_hcm_sub_idx = 0U;
    Sokoban_Actions_To_Waypoints(s_hcm_solution.sub_solutions[0].actions,
                                 s_hcm_solution.sub_solutions[0].count,
                                 start_pos,
                                 &s_hcm_waypoints);
    s_hcm_wp_idx = 0U;
    s_hcm_navigating = 0U;
    s_hcm_solve_ok = 1U;

    printf("HCM_SOLVED=1 boxes=%d wp0=%d\n",
           (int)s_hcm_solution.total_boxes,
           (int)s_hcm_waypoints.count);
}

/*
 * 主循环 5ms tick 总入口
 */
static void main_run_hardcoded_map_5ms(void)
{
    switch (s_hcm_phase)
    {
    case HCM_PHASE_SOLVE:
        /* 解算在 main() 初始化阶段已同步完成, 此处仅做一次性跳转 */
        if (s_hcm_solve_ok)
        {
            s_hcm_phase = HCM_PHASE_WARMUP;
            s_hcm_warmup_ticks = 0U;
            printf("HCM_WARMUP start\n");
        }
        /* 解算失败则永远停在此状态, 不开车 */
        return;

    case HCM_PHASE_WARMUP:
        /* 暖机: 等 IMU/编码器稳定 */
        if (++s_hcm_warmup_ticks >= MAIN_POINT_NAV_WARMUP_TICKS)
        {
            printf("HCM_WARMUP_DONE launch along y to solve start\n");
            s_hcm_phase = HCM_PHASE_LAUNCH_FIXED;
            chassis_ctrl_move_to_m(
                MAIN_POS_GRID_TO_M_X(MAIN_POS_HCM_LAUNCH_TARGET_X_GRID),
                MAIN_POS_GRID_TO_M_Y(MAIN_POS_HCM_LAUNCH_TARGET_Y_GRID),
                0.0f);
            s_hcm_navigating = 1U;
        }
        return;

    case HCM_PHASE_LAUNCH_FIXED:
        if (chassis_ctrl_is_arrived())
        {
            printf("HCM_LAUNCH_DONE start pushing\n");
            s_hcm_navigating = 0U;
            s_hcm_phase = HCM_PHASE_PUSH_BOXES;
        }
        return;

    case HCM_PHASE_PUSH_BOXES:
        if (!s_hcm_navigating)
        {
            /* 检查当前子箱航点是否已跑完 */
            if (s_hcm_wp_idx >= s_hcm_waypoints.count)
            {
                /* 当前子箱完成, 切下一子箱 */
                s_hcm_sub_idx++;
                if (s_hcm_sub_idx >= s_hcm_solution.total_boxes)
                {
                    /* 全部箱子推完 → 回发车区 */
                    printf("HCM_ALL_BOXES_DONE, returning home\n");
                    s_hcm_phase = HCM_PHASE_RETURN_HOME;
                    chassis_ctrl_move_to_m(
                        MAIN_POS_GRID_TO_M_X(MAIN_POS_HCM_HOME_X_GRID),
                        MAIN_POS_GRID_TO_M_Y(MAIN_POS_HCM_HOME_Y_GRID),
                        0.0f);
                    s_hcm_navigating = 1U;
                    return;
                }
                /* 装载下一子箱航点 */
                Sokoban_Actions_To_Waypoints(
                    s_hcm_solution.sub_solutions[s_hcm_sub_idx].actions,
                    s_hcm_solution.sub_solutions[s_hcm_sub_idx].count,
                    s_hcm_solution.player_end_pos[s_hcm_sub_idx - 1U],
                    &s_hcm_waypoints);
                s_hcm_wp_idx = 0U;
                printf("HCM_NEXT_BOX sub=%d wp=%d\n",
                       (int)s_hcm_sub_idx, (int)s_hcm_waypoints.count);
            }
            /* 派发当前航点 */
            {
                Point_t next = s_hcm_waypoints.points[s_hcm_wp_idx];
                chassis_ctrl_move_to_grid((uint8)next.x, (uint8)next.y);
                s_hcm_navigating = 1U;
            }
            return;
        }

        /* 已派发, 等到位 */
        if (!chassis_ctrl_is_arrived())
        {
            return;
        }
        s_hcm_navigating = 0U;
        s_hcm_wp_idx++;
        return;

    case HCM_PHASE_RETURN_HOME:
        if (chassis_ctrl_is_arrived())
        {
            chassis_ctrl_stop();
            printf("HCM_DONE returned to launch zone\n");
            s_hcm_phase = HCM_PHASE_DONE;
        }
        return;

    case HCM_PHASE_DONE:
        /* 永久驻停 */
        return;

    default:
        return;
    }
}

/* 50ms 调试打印 */
static void main_run_hardcoded_map_log_50ms(void)
{
    static uint8 s_div = 0U;
    if (++s_div < 10U) return;
    s_div = 0U;
    {
        chassis_pose_t pose = chassis_ctrl_get_pose();
        float tgt_x = 0.0f, tgt_y = 0.0f;
        chassis_ctrl_get_point_nav_target_m(&tgt_x, &tgt_y);
        printf("HCM: phase=%d sub=%d/%d wp=%d/%d pos=%.3f,%.3f tgt=%.3f,%.3f arr=%d\n",
               (int)s_hcm_phase,
               (int)s_hcm_sub_idx, (int)(s_hcm_solve_ok ? s_hcm_solution.total_boxes : 0),
               (int)s_hcm_wp_idx, (int)s_hcm_waypoints.count,
               pose.x_m, pose.y_m,
               tgt_x, tgt_y,
               (int)chassis_ctrl_is_arrived());
    }
}

#endif /* MAIN_RUN_MODE_HARDCODED_MAP */

/* ============================================================================
 *  STATIC_VERIFY mode (6): use one local 12x16 map, solve it once, then replay
 *  every Sokoban action on the IPS map monitor. The chassis never moves.
 * ============================================================================ */
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_STATIC_VERIFY)

typedef enum {
    M6_PHASE_SOLVE = 0,
    M6_PHASE_PLAY,
    M6_PHASE_RETURN,
    M6_PHASE_DONE,
    M6_PHASE_FAIL
} m6_phase_e;

#define M6_ACTION_STEP_TICKS     (60U)    /* 60 * 5ms = 300ms per action */
#define M6_VIS_ROWS              (MAP_ROWS)
#define M6_VIS_COLS              (MAP_COLS)

#if (M6_VIS_ROWS != MAP_ROWS) || (M6_VIS_COLS != MAP_COLS)
#error "M6 screenshot map must be 12x16."
#endif

/* Screenshot map: 12 rows x 16 cols. */
static const char s_m6_static_map[M6_VIS_ROWS][M6_VIS_COLS + 1] = {
    "################",
    "#-#---#--#----.#",
    "#---#--#-####--#",
    "#---#--#-------#",
    "#-.#-#-#--#----#",
    "#@-#-#--------##",
    "#--#----------##",
    "#-*----------*-#",
    "#--$-#----##$*-#",
    "#-$--#----####-#",
    "#----#.--------#",
    "################"
};

static SokoFullSolution_t s_m6_solution;
static uint8             s_m6_base_map[MAP_ROWS][MAP_COLS];
static uint8             s_m6_anim_map[MAP_ROWS][MAP_COLS];
static Point_t           s_m6_player;
static Point_t           s_m6_start;
static m6_phase_e        s_m6_phase = M6_PHASE_SOLVE;
static uint8             s_m6_solve_ok = 0U;
static uint8             s_m6_sub_idx = 0U;
static uint16            s_m6_act_idx = 0U;
static uint16            s_m6_step_ticks = 0U;
static uint16            s_m6_done_actions = 0U;
static uint16            s_m6_total_actions = 0U;
static NavPath_t         s_m6_return_path;
static uint16            s_m6_return_idx = 0U;

static uint8 main_m6_char_to_map(char ch)
{
    switch (ch)
    {
    case '#': return MAP_WALL;
    case '.': return MAP_TARGET;
    case '$': return MAP_BOX;
    case '*': return MAP_BOMB;
    case '@': return MAP_EMPTY;
    case '-':
    default:  return MAP_EMPTY;
    }
}

static void main_m6_load_static_map(void)
{
    uint8 r, c;
    uint8 vr, vc;

    s_m6_start.x = 1;
    s_m6_start.y = 1;

    for (r = 0U; r < MAP_ROWS; ++r)
    {
        for (c = 0U; c < MAP_COLS; ++c)
        {
            s_m6_anim_map[r][c] = MAP_EMPTY;
            s_m6_base_map[r][c] = MAP_EMPTY;
        }
    }

    for (vr = 0U; vr < M6_VIS_ROWS; ++vr)
    {
        for (vc = 0U; vc < M6_VIS_COLS; ++vc)
        {
            char ch = s_m6_static_map[vr][vc];
            uint8 cell = main_m6_char_to_map(ch);
            r = vr;
            c = vc;

            if (ch == '@')
            {
                s_m6_start.x = (int8)c;
                s_m6_start.y = (int8)r;
            }

            s_m6_anim_map[r][c] = cell;
            s_m6_base_map[r][c] = (cell == MAP_BOX) ? MAP_EMPTY : cell;
        }
    }

    s_m6_player = s_m6_start;
}

static void main_m6_publish_screen_state(void)
{
    app_link_inject_static_map(s_m6_anim_map);
    app_link_inject_static_car((uint8)s_m6_player.x, (uint8)s_m6_player.y);
}

static void main_m6_solve(void)
{
    uint8 i;

    main_m6_load_static_map();
    memset(&s_m6_solution, 0, sizeof(s_m6_solution));

    if (!Sokoban_Solve_Stage1(s_m6_anim_map, s_m6_start, &s_m6_solution) ||
        (s_m6_solution.is_solved == 0U) ||
        (s_m6_solution.total_boxes == 0U))
    {
        s_m6_solve_ok = 0U;
        s_m6_phase = M6_PHASE_FAIL;
        main_m6_publish_screen_state();
        return;
    }

    s_m6_total_actions = 0U;
    for (i = 0U; i < s_m6_solution.total_boxes; ++i)
    {
        s_m6_total_actions = (uint16)(s_m6_total_actions +
                                      s_m6_solution.sub_solutions[i].count);
    }

    s_m6_sub_idx = 0U;
    s_m6_act_idx = 0U;
    s_m6_step_ticks = 0U;
    s_m6_done_actions = 0U;
    s_m6_solve_ok = 1U;
    s_m6_phase = M6_PHASE_PLAY;
    main_m6_publish_screen_state();
}

static uint8 main_m6_action_delta(SokoAction_e act, int8 *dx, int8 *dy)
{
    *dx = 0;
    *dy = 0;

    switch (act)
    {
    case SOKO_ACT_UP:    *dy = -1; return 1U;
    case SOKO_ACT_DOWN:  *dy =  1; return 1U;
    case SOKO_ACT_LEFT:  *dx = -1; return 1U;
    case SOKO_ACT_RIGHT: *dx =  1; return 1U;
    default: return 0U;
    }
}

static uint8 main_m6_point_in_map(Point_t p)
{
    return (uint8)((p.x >= 0) && (p.x < (int8)MAP_COLS) &&
                   (p.y >= 0) && (p.y < (int8)MAP_ROWS));
}

static void main_m6_plan_return_home(void)
{
    memset(&s_m6_return_path, 0, sizeof(s_m6_return_path));
    s_m6_return_idx = 0U;

    if (!Algo_Nav_BFS(s_m6_anim_map, s_m6_player, s_m6_start, &s_m6_return_path))
    {
        s_m6_phase = M6_PHASE_FAIL;
        return;
    }

    s_m6_phase = (s_m6_return_path.step_count == 0U) ? M6_PHASE_DONE : M6_PHASE_RETURN;
}

static void main_m6_apply_next_action(void)
{
    SokoActionSeq_t *seq;
    SokoAction_e act;
    int8 dx, dy;
    Point_t next;

    if ((s_m6_phase != M6_PHASE_PLAY) ||
        (s_m6_sub_idx >= s_m6_solution.total_boxes))
    {
        return;
    }

    seq = &s_m6_solution.sub_solutions[s_m6_sub_idx];
    if (s_m6_act_idx >= seq->count)
    {
        s_m6_sub_idx++;
        s_m6_act_idx = 0U;
        if (s_m6_sub_idx >= s_m6_solution.total_boxes)
        {
            main_m6_plan_return_home();
        }
        return;
    }

    act = seq->actions[s_m6_act_idx];
    if (main_m6_action_delta(act, &dx, &dy) == 0U)
    {
        s_m6_phase = M6_PHASE_FAIL;
        return;
    }

    next.x = (int8)(s_m6_player.x + dx);
    next.y = (int8)(s_m6_player.y + dy);
    if (main_m6_point_in_map(next) == 0U)
    {
        s_m6_phase = M6_PHASE_FAIL;
        return;
    }

    if (s_m6_anim_map[next.y][next.x] == MAP_BOX)
    {
        Point_t box_next;
        box_next.x = (int8)(next.x + dx);
        box_next.y = (int8)(next.y + dy);
        if ((main_m6_point_in_map(box_next) == 0U) ||
            (s_m6_anim_map[box_next.y][box_next.x] == MAP_WALL) ||
            (s_m6_anim_map[box_next.y][box_next.x] == MAP_BOX))
        {
            s_m6_phase = M6_PHASE_FAIL;
            return;
        }

        s_m6_anim_map[next.y][next.x] = s_m6_base_map[next.y][next.x];
        if (s_m6_base_map[box_next.y][box_next.x] == MAP_TARGET)
        {
            s_m6_anim_map[box_next.y][box_next.x] = MAP_TARGET;
        }
        else
        {
            s_m6_anim_map[box_next.y][box_next.x] = MAP_BOX;
        }
    }
    else if (s_m6_anim_map[next.y][next.x] == MAP_WALL)
    {
        s_m6_phase = M6_PHASE_FAIL;
        return;
    }

    s_m6_player = next;
    s_m6_act_idx++;
    s_m6_done_actions++;
}

static void main_run_static_verify_5ms(void)
{
    chassis_ctrl_stop();

    if (s_m6_phase == M6_PHASE_SOLVE)
    {
        main_m6_solve();
        return;
    }

    if (s_m6_phase == M6_PHASE_PLAY)
    {
        if (++s_m6_step_ticks >= M6_ACTION_STEP_TICKS)
        {
            s_m6_step_ticks = 0U;
            main_m6_apply_next_action();
        }
    }
    else if (s_m6_phase == M6_PHASE_RETURN)
    {
        if (++s_m6_step_ticks >= M6_ACTION_STEP_TICKS)
        {
            s_m6_step_ticks = 0U;
            if (s_m6_return_idx < s_m6_return_path.step_count)
            {
                s_m6_player = s_m6_return_path.path[s_m6_return_idx];
                s_m6_return_idx++;
            }
            if (s_m6_return_idx >= s_m6_return_path.step_count)
            {
                s_m6_phase = M6_PHASE_DONE;
            }
        }
    }

    main_m6_publish_screen_state();
}

static void main_mode6_render_100ms(void)
{
    const char *phase_str;
    uint16 y;
    uint8 seg_show;

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0U, 188U, "                                        ");
    ips200_show_string(0U, 206U, "                                        ");

    switch (s_m6_phase)
    {
    case M6_PHASE_SOLVE: phase_str = "M6 STATIC: SOLVING"; break;
    case M6_PHASE_PLAY:  phase_str = "M6 STATIC: PLAYING"; break;
    case M6_PHASE_DONE:  phase_str = "M6 STATIC: DONE";    break;
    case M6_PHASE_FAIL:
    default:             phase_str = "M6 STATIC: FAIL";    break;
    }

    y = 188U;
    ips200_set_color((s_m6_phase == M6_PHASE_FAIL) ? RGB565_RED : RGB565_CYAN, RGB565_BLACK);
    ips200_show_string(0U, y, (char *)phase_str);

    if (s_m6_solve_ok != 0U)
    {
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        ips200_show_string(196U, y, "BOX:");
        ips200_show_uint(236U, y, s_m6_solution.total_boxes, 1U);
    }

    y = 206U;
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    if (s_m6_solve_ok != 0U)
    {
        seg_show = s_m6_sub_idx;
        if (seg_show >= s_m6_solution.total_boxes)
        {
            seg_show = (uint8)(s_m6_solution.total_boxes - 1U);
        }

        ips200_show_string(0U, y, "ACT");
        ips200_show_uint(32U, y, s_m6_done_actions, 3U);
        ips200_show_string(62U, y, "/");
        ips200_show_uint(72U, y, s_m6_total_actions, 3U);
        ips200_show_string(112U, y, "SEG");
        ips200_show_uint(144U, y, (uint32)(seg_show + 1U), 1U);
        ips200_show_string(158U, y, "/");
        ips200_show_uint(168U, y, s_m6_solution.total_boxes, 1U);
        ips200_show_string(198U, y, "300ms/act");
    }
    else
    {
        ips200_set_color(RGB565_RED, RGB565_BLACK);
        ips200_show_string(0U, y, "No solution for static map");
    }
}

#endif /* MAIN_RUN_MODE_STATIC_VERIFY */

/* ============================================================================
 *  SOLVE_VERIFY 模式 (5): 算法解算验证
 *  流程:
 *    1. WAIT_MAP     — 等待 OpenART 通过 UART4 发来地图帧
 *    2. SOLVE        — 收到地图后立即解算推箱子路径 (Stage1), 同时关 UART4 RX
 *    3. WAIT_LEAVE   — 模拟人工发车等待 (定时器, 2s)
 *    4. WARMUP       — 模拟 IMU/编码器暖机 (定时器, 1s)
 *    5. PUSH_BOXES   — 逐子箱逐航点虚拟推箱 (定时器推进, 1s/航点)
 *    6. RETURN_HOME  — 模拟回到发车区 (定时器, 1s)
 *    7. DONE         — 验证完成
 *
 *  车全程不动 (chassis_ctrl_stop), 仅验证算法能否正确解算摄像头识别的真实地图.
 *  地图色块由 chassis_menu_render_100ms() 渲染,
 *  解算进度由 main_mode5_render_100ms() 叠加在屏幕下部文字区域.
 * ============================================================================ */
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_SOLVE_VERIFY)

typedef enum {
    SV_PHASE_WAIT_MAP = 0,   /* 等待摄像头地图帧                   */
    SV_PHASE_SOLVE,          /* 求解 (同一 tick 内完成)             */
    SV_PHASE_WAIT_LEAVE,     /* 模拟发车: 等待推出发车区 (2s 定时)  */
    SV_PHASE_WARMUP,         /* 模拟暖机: 等待 IMU/编码器稳定 (1s)  */
    SV_PHASE_PUSH_BOXES,     /* 虚拟推箱: 逐航点定时推进 (1s/步)    */
    SV_PHASE_RETURN_HOME,    /* 模拟回发车区 (1s 定时)              */
    SV_PHASE_DONE            /* 验证完成, 永久驻停                   */
} sv_phase_e;

/* 各阶段虚拟定时时长 (单位: 5ms tick) */
#define SV_WAIT_LEAVE_TICKS    (400U)   /* 模拟发车 2s               */
#define SV_WARMUP_TICKS        (200U)   /* 模拟暖机 1s               */
#define SV_WP_STEP_TICKS       (200U)   /* 每航点虚拟推箱 1s         */
#define SV_RETURN_TICKS        (200U)   /* 模拟回库 1s               */

static SokoFullSolution_t  s_sv_solution;
static SokoWaypointPath_t  s_sv_waypoints;
static uint8               s_sv_map[MAP_ROWS][MAP_COLS];
static sv_phase_e          s_sv_phase        = SV_PHASE_WAIT_MAP;
static uint8               s_sv_solve_ok     = 0U;
static uint8               s_sv_sub_idx      = 0U;   /* 当前子箱索引       */
static uint16              s_sv_wp_idx       = 0U;   /* 当前航点索引       */
static uint16              s_sv_phase_ticks  = 0U;   /* 当前阶段已过 tick  */
static uint32              s_sv_map_recv_ms  = 0U;

/*
 * 拷贝 app_link 地图快照并求解, 结果写入静态变量.
 * 解算失败则 s_sv_solve_ok = 0, 状态机停在 SOLVE 阶段, 屏幕显示失败.
 */
static void main_sv_solve(void)
{
    Point_t start_pos;

    start_pos.x = (int8)MAIN_POS_NAV_START_X_GRID;
    start_pos.y = (int8)MAIN_POS_NAV_START_Y_GRID;

    app_link_get_map_snapshot(s_sv_map);

    memset(&s_sv_solution, 0, sizeof(s_sv_solution));
    if (!Sokoban_Solve_Stage1(s_sv_map, start_pos, &s_sv_solution))
    {
        s_sv_solve_ok = 0U;
        return;
    }
    if (!s_sv_solution.is_solved || s_sv_solution.total_boxes == 0U)
    {
        s_sv_solve_ok = 0U;
        return;
    }

    /* 装载首子箱航点 */
    s_sv_sub_idx = 0U;
    Sokoban_Actions_To_Waypoints(s_sv_solution.sub_solutions[0].actions,
                                 s_sv_solution.sub_solutions[0].count,
                                 start_pos,
                                 &s_sv_waypoints);
    s_sv_wp_idx = 0U;
    s_sv_solve_ok = 1U;
}

/*
 * 主循环 5ms 钩子: 驱动虚拟状态机 (WAIT_MAP → SOLVE → … → DONE).
 * 车不驱动, 仅用定时器模拟各阶段时长.
 */
static void main_run_solve_verify_5ms(void)
{
    switch (s_sv_phase)
    {
    case SV_PHASE_WAIT_MAP:
    {
        uint32 now_ms = g_link_last_map_ms;
        if (now_ms != 0U && now_ms != s_sv_map_recv_ms)
        {
            s_sv_map_recv_ms = now_ms;
            s_sv_phase = SV_PHASE_SOLVE;
        }
    }
    return;

    case SV_PHASE_SOLVE:
        /* 立即求解, 锁定地图不再接收新帧 */
        uart_rx_interrupt(MAIN_OPENART1_UART, 0);
        main_sv_solve();
        if (s_sv_solve_ok)
        {
            s_sv_phase        = SV_PHASE_WAIT_LEAVE;
            s_sv_phase_ticks  = 0U;
        }
        /* 解算失败: 停在 SOLVE 阶段, 屏幕显示失败信息 */
        return;

    case SV_PHASE_WAIT_LEAVE:
        if (++s_sv_phase_ticks >= SV_WAIT_LEAVE_TICKS)
        {
            s_sv_phase        = SV_PHASE_WARMUP;
            s_sv_phase_ticks  = 0U;
        }
        return;

    case SV_PHASE_WARMUP:
        if (++s_sv_phase_ticks >= SV_WARMUP_TICKS)
        {
            s_sv_phase        = SV_PHASE_PUSH_BOXES;
            s_sv_phase_ticks  = 0U;
        }
        return;

    case SV_PHASE_PUSH_BOXES:
        if (++s_sv_phase_ticks >= SV_WP_STEP_TICKS)
        {
            s_sv_phase_ticks = 0U;
            s_sv_wp_idx++;
            if (s_sv_wp_idx >= s_sv_waypoints.count)
            {
                /* 当前子箱走完, 切下一子箱 */
                s_sv_sub_idx++;
                if (s_sv_sub_idx >= s_sv_solution.total_boxes)
                {
                    /* 全部箱子推完 → 模拟返回发车区 */
                    s_sv_phase        = SV_PHASE_RETURN_HOME;
                    s_sv_phase_ticks  = 0U;
                    return;
                }
                /* 装载下一子箱航点 */
                Sokoban_Actions_To_Waypoints(
                    s_sv_solution.sub_solutions[s_sv_sub_idx].actions,
                    s_sv_solution.sub_solutions[s_sv_sub_idx].count,
                    s_sv_solution.player_end_pos[s_sv_sub_idx - 1U],
                    &s_sv_waypoints);
                s_sv_wp_idx = 0U;
            }
        }
        return;

    case SV_PHASE_RETURN_HOME:
        if (++s_sv_phase_ticks >= SV_RETURN_TICKS)
        {
            s_sv_phase = SV_PHASE_DONE;
        }
        return;

    case SV_PHASE_DONE:
    default:
        return;
    }
}

/*
 * 100ms 渲染: 在菜单地图下方叠加解算进度文字.
 * 必须在 chassis_menu_render_100ms() 之后调用, 避免被 ips200_full() 刷掉.
 * 复用菜单的 y=188 (状态行) 和 y=206 (详情行), 车不动故无需显示位姿.
 */
static void main_mode5_render_100ms(void)
{
    uint16 y;
    const char *phase_str;

    /* 清空两行 (菜单写了 car/pose 信息, 这里用解算信息覆盖) */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0U, 188U, "                                        ");
    ips200_show_string(0U, 206U, "                                        ");

    /* ---- 行1 (y=188): 阶段名 + 解算结果 ---- */
    y = 188U;
    ips200_set_color(RGB565_CYAN, RGB565_BLACK);
    switch (s_sv_phase)
    {
    case SV_PHASE_WAIT_MAP:   phase_str = "PHASE: WAIT MAP...";          break;
    case SV_PHASE_SOLVE:      phase_str = "PHASE: SOLVING...";           break;
    case SV_PHASE_WAIT_LEAVE: phase_str = "PHASE: WAIT LEAVE (2s sim)";  break;
    case SV_PHASE_WARMUP:     phase_str = "PHASE: WARMUP (1s sim)";      break;
    case SV_PHASE_PUSH_BOXES: phase_str = "PHASE: PUSH BOXES (1s/wp)";   break;
    case SV_PHASE_RETURN_HOME:phase_str = "PHASE: RETURN HOME (1s sim)"; break;
    case SV_PHASE_DONE:       phase_str = "PHASE: DONE - VERIFIED";      break;
    default:                  phase_str = "???";                         break;
    }
    ips200_show_string(0U, y, (char *)phase_str);

    /* 解算状态: OK + 箱子数, 或 NO SOLUTION */
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    if (s_sv_phase >= SV_PHASE_SOLVE && s_sv_solve_ok)
    {
        ips200_set_color(RGB565_GREEN, RGB565_BLACK);
        ips200_show_string(210U, y, "OK BOX:");
        ips200_show_uint(270U, y, s_sv_solution.total_boxes, 2U);
    }
    else if (s_sv_phase >= SV_PHASE_SOLVE)
    {
        ips200_set_color(RGB565_RED, RGB565_BLACK);
        ips200_show_string(210U, y, "NO SOLUTION!");
    }

    /* ---- 行2 (y=206): 进度详情 ---- */
    y = 206U;
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    if (s_sv_solve_ok && s_sv_phase >= SV_PHASE_PUSH_BOXES)
    {
        /* SEG 1/3  WP 02/15 */
        ips200_show_string(0U,   y, "SEG");
        ips200_show_uint( 40U,  y, (uint32)(s_sv_sub_idx + 1U), 1U);
        ips200_show_string(54U,  y, "/");
        ips200_show_uint( 64U,  y, s_sv_solution.total_boxes, 1U);
        ips200_show_string(88U,  y, "WP");
        ips200_show_uint( 112U, y, (uint32)(s_sv_wp_idx + 1U), 2U);
        ips200_show_string(134U, y, "/");
        ips200_show_uint( 144U, y, (uint32)s_sv_waypoints.count, 2U);
        if (s_sv_phase == SV_PHASE_DONE)
        {
            ips200_set_color(RGB565_GREEN, RGB565_BLACK);
            ips200_show_string(180U, y, "ALL DONE");
        }
    }
    else if (s_sv_solve_ok && s_sv_phase == SV_PHASE_WAIT_LEAVE)
    {
        ips200_show_string(0U, y, "(sim) waiting for car to leave launch zone...");
    }
    else if (s_sv_solve_ok && s_sv_phase == SV_PHASE_WARMUP)
    {
        ips200_show_string(0U, y, "(sim) warming up IMU / encoders...");
    }
    else if (!s_sv_solve_ok && s_sv_phase >= SV_PHASE_SOLVE)
    {
        ips200_set_color(RGB565_RED, RGB565_BLACK);
        ips200_show_string(0U, y, "Check: map format / camera connection / algorithm");
    }
}

#endif /* MAIN_RUN_MODE_SOLVE_VERIFY */

/* ============================================================
 * 位置闭环坐标系 (POINT_NAV 模式) — 整数格约定
 *   单位: 1 格步长 (X = 3.2m / 14 ≈ 0.2286m, Y = 2.4m / 10 = 0.24m)
 *   范围: X ∈ [0, 14], Y ∈ [0, 10]   (0 = 左/上边界, 14/10 = 右/下边界)
 *   起点: (0.5, 5.5) 格 — 发车区中心, 距左墙半格, 距上墙 5.5 格
 *
 *   目标坐标传整数即可, 内部按 grid * step 自动换算成米送入闭环.
 *   起点用半整数 (0.5, 5.5) 描述发车区中心位置, 与硬件实测吻合.
 * ============================================================ */
/* 注: MAIN_POS_NAV_START_X_GRID / MAIN_POS_NAV_START_Y_GRID 已上提到文件
 * 顶部 SOKO_SELFTEST 宏前, 避免静态地图模式编译期前向引用. */

/* >>>>>>>>>>>> 改这两行换目标格 <<<<<<<<<<<< */
#define MAIN_POS_NAV_TARGET_X_GRID    (14)     /* 整数 0..14, 14 = 右边界 */
#define MAIN_POS_NAV_TARGET_Y_GRID    (10)     /* 整数 0..10, 10 = 下边界 */
/* <<<<<<<<<<<< 改这两行换目标格 >>>>>>>>>>>> */

#define MAIN_POS_NAV_HOLD_YAW_DEG     (180.0f)   /* 全程锁住 0° 航向 */

/*
 * 上电暖机等待时长 (5ms tick 数). 200 × 5ms = 1s.
 * 等待以下子系统稳定后, 主循环才下发第1个导航目标:
 *   1. IMU KF 收敛 (~500ms): Kalman P 压缩到稳态, 零偏估计准确;
 *      未收敛时 yaw 漂移 1~5°, 里程计误差随时间线性累积.
 *   2. 编码器 LPF 稳定 (~100ms): 前几拍速度反馈偏大, odometry 的 vx/vy 有初始误差.
 *   3. 静止检测滑窗填满 (STILL_WINDOW_LEN × 5ms): 窗口未满时在线零偏自适应不工作.
 * 暖机期 chassis_ctrl 保持 YAW_HOLD, 位置积分 s_pos_i 不运行, 无积分超调风险.
 */
/* MAIN_POINT_NAV_WARMUP_TICKS is defined near the shared position macros above. */

/* 格 → 米换算 (POINT_NAV 专用, 不影响 BFS 的内场索引体系).
 * 约定: 整数 n = 第 n 格中心 (1-based), 公式为 (n - 0.5) × STEP.
 *   n=1  → 0.5 × STEP (第1格中心, 距左/上墙半格)
 *   n=14 → 13.5 × STEP_X = 3.086m (第14格中心, 距右墙 0.114m, 可到达)
 *   n=10 → 9.5  × STEP_Y = 2.28m  (第10格中心, 距下墙 0.12m,  可到达)
 * 注: 若传入半整数 5.5 则 (5.5-0.5)×STEP = 5×STEP = 第5/6格边界, 用于起点描述. */
/* MAIN_POS_GRID_TO_M_X / MAIN_POS_GRID_TO_M_Y 已上提到文件顶部, 此处仅留
 * TARGET 米坐标派生宏以便阅读. */
#define MAIN_POS_NAV_TARGET_X_M       MAIN_POS_GRID_TO_M_X(MAIN_POS_NAV_TARGET_X_GRID)
#define MAIN_POS_NAV_TARGET_Y_M       MAIN_POS_GRID_TO_M_Y(MAIN_POS_NAV_TARGET_Y_GRID)

/* ========================================================================== */
/*  ⬇⬇⬇ 以下是单轮 PID 调试专用代码, 仅在 MAIN_RUN_MODE_SINGLE_WHEEL 生效 ⬇⬇⬇  */
/*  ⬇⬇⬇ 姿态闭环调试阶段这里全部被 #if 屏蔽, 不会被编译, 不要删 ⬇⬇⬇          */
/* ========================================================================== */
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_SINGLE_WHEEL)
#define MAIN_PID_DEBUG_WHEEL_INDEX    (CHASSIS_WHEEL_RB)  /* 0=LF, 1=RF, 2=LB, 3=RB */
#define MAIN_PID_DEBUG_TARGET_MPS     (0.10f)             /* target wheel speed, m/s */
#define MAIN_PID_DEBUG_FORCE_PID      (1)                 /* 1=use KP/KI/KD below */
#define MAIN_PID_DEBUG_KP             (70.0f)
#define MAIN_PID_DEBUG_KI             (20.0f)
#define MAIN_PID_DEBUG_KD             (0.0f)
#endif /* MAIN_RUN_MODE_SINGLE_WHEEL */
/* ========================================================================== */
/*  ⬆⬆⬆ 单轮 PID 调试专用代码结束 ⬆⬆⬆                                            */
/* ========================================================================== */

/* 菜单渲染分频 (主循环 5ms tick * N), N=20 => 100ms 刷一次 */
#if (CHASSIS_MENU_ENABLE != 0)
#define MAIN_MENU_RENDER_DIV          (20U)
#endif

#if (MAIN_RUN_MODE == MAIN_RUN_MODE_OPENART2_TEST)
#define OA2_TEST_STALE_MS             (500U)

typedef enum
{
    OA2_PHASE_WAIT_LINK = 0,
    OA2_PHASE_WAIT_FRAME,
    OA2_PHASE_SHOW_BOX,
    OA2_PHASE_SHOW_TARGET,
    OA2_PHASE_SHOW_NONE,
    OA2_PHASE_STALE
} oa2_test_phase_e;

static oa2_test_phase_e s_oa2_phase = OA2_PHASE_WAIT_LINK;
static uint32 s_oa2_last_frame_id = 0U;
static uint32 s_oa2_box_count = 0U;
static uint32 s_oa2_target_count = 0U;
static uint32 s_oa2_none_count = 0U;

static const char *main_oa2_phase_name(oa2_test_phase_e phase)
{
    switch (phase)
    {
    case OA2_PHASE_WAIT_LINK:   return "WAIT LINK";
    case OA2_PHASE_WAIT_FRAME:  return "WAIT FRAME";
    case OA2_PHASE_SHOW_BOX:    return "BOX";
    case OA2_PHASE_SHOW_TARGET: return "TARGET";
    case OA2_PHASE_SHOW_NONE:   return "NONE";
    case OA2_PHASE_STALE:
    default:                    return "STALE";
    }
}

static const char *main_oa2_kind_name(uint8 kind, uint8 class_id)
{
    if (class_id == APP_LINK_CLASS_ID_NONE)
    {
        return "NONE";
    }
    if (kind == APP_LINK_OBJ_KIND_BOX)
    {
        return "BOX";
    }
    if (kind == APP_LINK_OBJ_KIND_TARGET)
    {
        return "TARGET";
    }
    return "BAD";
}

static void main_run_openart2_test_5ms(void)
{
    app_link_box_class_snapshot_t snap;
    uint32 now_ms = app_link_get_ms();
    uint32 link_ms = g_link_last_class_link_ms;

    chassis_ctrl_stop();
    app_link_get_box_class_snapshot(&snap);

    if (link_ms == 0U)
    {
        s_oa2_phase = OA2_PHASE_WAIT_LINK;
        return;
    }
    if ((now_ms - link_ms) > OA2_TEST_STALE_MS)
    {
        s_oa2_phase = OA2_PHASE_STALE;
        return;
    }
    if (snap.valid == 0U)
    {
        s_oa2_phase = OA2_PHASE_WAIT_FRAME;
        return;
    }
    if ((now_ms - snap.stamp_ms) > OA2_TEST_STALE_MS)
    {
        s_oa2_phase = OA2_PHASE_STALE;
        return;
    }

    if (snap.frame_id != s_oa2_last_frame_id)
    {
        s_oa2_last_frame_id = snap.frame_id;
        if (snap.class_id == APP_LINK_CLASS_ID_NONE)
        {
            s_oa2_none_count++;
        }
        else if (snap.obj_kind == APP_LINK_OBJ_KIND_BOX)
        {
            s_oa2_box_count++;
        }
        else if (snap.obj_kind == APP_LINK_OBJ_KIND_TARGET)
        {
            s_oa2_target_count++;
        }
    }

    if (snap.class_id == APP_LINK_CLASS_ID_NONE)
    {
        s_oa2_phase = OA2_PHASE_SHOW_NONE;
    }
    else if (snap.obj_kind == APP_LINK_OBJ_KIND_BOX)
    {
        s_oa2_phase = OA2_PHASE_SHOW_BOX;
    }
    else if (snap.obj_kind == APP_LINK_OBJ_KIND_TARGET)
    {
        s_oa2_phase = OA2_PHASE_SHOW_TARGET;
    }
    else
    {
        s_oa2_phase = OA2_PHASE_STALE;
    }
}

static void main_openart2_test_render_100ms(void)
{
    app_link_box_class_snapshot_t snap;
    uint32 now_ms = app_link_get_ms();
    uint32 link_age = (g_link_last_class_link_ms == 0U) ? 0U : (now_ms - g_link_last_class_link_ms);
    uint32 frame_age = 0U;
    uint16 title_color = RGB565_CYAN;

    app_link_get_box_class_snapshot(&snap);
    if (snap.valid != 0U)
    {
        frame_age = now_ms - snap.stamp_ms;
    }

    if ((s_oa2_phase == OA2_PHASE_SHOW_BOX) || (s_oa2_phase == OA2_PHASE_SHOW_TARGET))
    {
        title_color = RGB565_GREEN;
    }
    else if ((s_oa2_phase == OA2_PHASE_STALE) || (s_oa2_phase == OA2_PHASE_WAIT_LINK))
    {
        title_color = RGB565_RED;
    }
    else
    {
        title_color = RGB565_YELLOW;
    }

    ips200_full(RGB565_BLACK);
    ips200_set_color(title_color, RGB565_BLACK);
    ips200_show_string(0U, 0U, "OPENART2 CLASS TEST");

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(0U, 24U, "STATE:");
    ips200_show_string(64U, 24U, (char *)main_oa2_phase_name(s_oa2_phase));

    ips200_show_string(0U, 48U, "UART1 AGE:");
    ips200_show_uint(96U, 48U, link_age, 5U);
    ips200_show_string(144U, 48U, "ms");

    ips200_show_string(0U, 72U, "FRAME AGE:");
    if (snap.valid != 0U)
    {
        ips200_show_uint(96U, 72U, frame_age, 5U);
        ips200_show_string(144U, 72U, "ms");
    }
    else
    {
        ips200_show_string(96U, 72U, "----- ms");
    }

    ips200_show_string(0U, 104U, "OBJ:");
    if (snap.valid != 0U)
    {
        ips200_show_string(48U, 104U, (char *)main_oa2_kind_name(snap.obj_kind, snap.class_id));
    }
    else
    {
        ips200_show_string(48U, 104U, "--");
    }

    ips200_show_string(0U, 128U, "CLASS:");
    if ((snap.valid != 0U) && (snap.class_id != APP_LINK_CLASS_ID_NONE))
    {
        ips200_show_uint(64U, 128U, snap.class_id, 2U);
    }
    else
    {
        ips200_show_string(64U, 128U, "--");
    }
    ips200_show_string(112U, 128U, "SEQ:");
    if (snap.valid != 0U)
    {
        ips200_show_uint(152U, 128U, snap.vision_seq, 3U);
    }
    else
    {
        ips200_show_string(152U, 128U, "---");
    }

    ips200_show_string(0U, 152U, "FID:");
    ips200_show_uint(48U, 152U, snap.valid ? snap.frame_id : 0U, 6U);

    ips200_show_string(0U, 176U, "CNT B:");
    ips200_show_uint(56U, 176U, s_oa2_box_count, 4U);
    ips200_show_string(112U, 176U, "T:");
    ips200_show_uint(136U, 176U, s_oa2_target_count, 4U);
    ips200_show_string(192U, 176U, "N:");
    ips200_show_uint(216U, 176U, s_oa2_none_count, 4U);

    ips200_show_string(0U, 208U, "OK:");
    ips200_show_uint(32U, 208U, g_link_class_stats.frames_ok, 5U);
    ips200_show_string(88U, 208U, "CRC:");
    ips200_show_uint(128U, 208U, g_link_class_stats.frames_crc_err, 4U);
    ips200_show_string(176U, 208U, "LEN:");
    ips200_show_uint(216U, 208U, g_link_class_stats.frames_len_err, 4U);
    ips200_show_string(264U, 208U, "SQ:");
    ips200_show_uint(288U, 208U, g_link_class_stats.frames_seq_drop, 3U);
}
#endif

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
 int main(void)
{
#if (CHASSIS_MENU_ENABLE != 0)
    uint8 menu_render_div = 0U;
#endif

    clock_init(BOARD_BOOTCLOCKRUN_CORE_CLOCK);  // 默认528MHz，降低电池冷启动的VDD_SOC需求
    debug_init();                   // 调试端口初始化

    // ------------------------------------------------------------------
    // 1. 通信外设初始化
    // ------------------------------------------------------------------
    // OpenART1: 地图识别模块, 通过 UART4 上报 MAP/HEARTBEAT 帧.
#if ((MAIN_RUN_MODE == MAIN_RUN_MODE_GAME) || (MAIN_RUN_MODE == MAIN_RUN_MODE_SOKO_SELFTEST) || (MAIN_RUN_MODE == MAIN_RUN_MODE_SOLVE_VERIFY) || (MAIN_RUN_MODE == MAIN_RUN_MODE_LEVEL1_TEST) || (MAIN_RUN_MODE == MAIN_RUN_MODE_LEVEL2_TEST))
    uart_init(MAIN_OPENART1_UART, 115200, MAIN_OPENART1_UART_TX, MAIN_OPENART1_UART_RX);
    uart_rx_interrupt(MAIN_OPENART1_UART, 1);
#endif

    // OpenART2: 分类识别模块, 通过 UART1 上报 BOX_CLASS/HEARTBEAT 帧.
#if ((MAIN_RUN_MODE == MAIN_RUN_MODE_GAME) || (MAIN_RUN_MODE == MAIN_RUN_MODE_OPENART2_TEST) || (MAIN_RUN_MODE == MAIN_RUN_MODE_LEVEL2_TEST))
    uart_init(MAIN_OPENART2_UART, 115200, MAIN_OPENART2_UART_TX, MAIN_OPENART2_UART_RX);
    uart_rx_interrupt(MAIN_OPENART2_UART, 1);
#endif

#if ((MAIN_RUN_MODE == MAIN_RUN_MODE_GAME) || (MAIN_RUN_MODE == MAIN_RUN_MODE_SOKO_SELFTEST) || (MAIN_RUN_MODE == MAIN_RUN_MODE_SOLVE_VERIFY) || (MAIN_RUN_MODE == MAIN_RUN_MODE_LEVEL1_TEST) || (MAIN_RUN_MODE == MAIN_RUN_MODE_OPENART2_TEST) || (MAIN_RUN_MODE == MAIN_RUN_MODE_LEVEL2_TEST))
    app_link_init();                /* P0-1: 协议解析层初始化, 必须在 uart_rx_interrupt 之后 */
#endif

    // ------------------------------------------------------------------
    // 2. IPS200 屏幕 + 按键初始化 (调参菜单)
    // ------------------------------------------------------------------
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_STATIC_VERIFY)
    app_link_init();
#endif

#if (CHASSIS_MENU_ENABLE != 0)
    /*
     * IPS200 启动诊断: 复位后等 200ms (ST7789 要求≥120ms), 清一次黑屏.
     * 仅此一次写入, 后续主循环不写屏(HCM模式跳过 menu_render).
     * ─ 清屏成功→黑屏, SPI 正常
     * ─ 卡死在这里→屏幕排线松动/模组损坏, 硬件问题需排查
     */
    ips200_set_dir(IPS200_CROSSWISE);
    ips200_init(IPS200_TYPE_SPI);
    system_delay_ms(200);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_full(RGB565_BLACK);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    key_init(10);
#endif

    // ------------------------------------------------------------------
    // 3. 底盘控制子系统初始化 + 调参菜单
    //    包括: IMU 零偏标定、电机 PWM、编码器、PID 参数
    //    注意: 调用此函数前请确保车模静止放置在平面上
    // ------------------------------------------------------------------
    chassis_ctrl_init();
#if (CHASSIS_MENU_ENABLE != 0)
    chassis_menu_init();
#endif
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_LEVEL1_TEST) || (MAIN_RUN_MODE == MAIN_RUN_MODE_HARDCODED_MAP) || (MAIN_RUN_MODE == MAIN_RUN_MODE_LEVEL2_TEST)
    /* 模式7/8固定发车: 上电先把里程计放在 (1,5.5), 暖机后沿 Y 轴平移到 (1,5). */
    chassis_ctrl_set_pose(MAIN_POS_GRID_TO_M_X(MAIN_POS_HCM_HOME_X_GRID),
                          MAIN_POS_GRID_TO_M_Y(MAIN_POS_HCM_HOME_Y_GRID),
                          0.0f);
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_GAME)
    /* 正式比赛: 发车区人工摆车时车头朝地图 -Y, 以 180° 写入全局 yaw 基准. */
    chassis_ctrl_set_pose(MAIN_POS_GRID_TO_M_X(MAIN_POS_NAV_START_X_GRID),
                          MAIN_POS_GRID_TO_M_Y(MAIN_POS_NAV_START_Y_GRID),
                          APP_GAME_LAUNCH_FACE_YAW_DEG);
#else
    /* 解算/导航起点: 整数格约定 (1, 5.5) → 自动换算成米送入里程计原点. */
    chassis_ctrl_set_pose(MAIN_POS_GRID_TO_M_X(MAIN_POS_NAV_START_X_GRID),
                          MAIN_POS_GRID_TO_M_Y(MAIN_POS_NAV_START_Y_GRID),
                          0.0f);
#endif

#if (MAIN_RUN_MODE == MAIN_RUN_MODE_SINGLE_WHEEL)
    /* ⬇⬇⬇ 单轮 PID 调试启动逻辑, 姿态调试阶段这里被 #if 屏蔽 ⬇⬇⬇ */
    /* 单轮 PID 调试模式: 仅一个轮子参与闭环, 其余轮子目标恒 0;
     * 闭环执行体在 PIT_CH1 / chassis_ctrl_task_20ms 内. */
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
    /* ✅ 姿态闭环调试走这里: 只设一次目标角, 后续 PIT_CH1 20ms 中断中持续闭环. */
    chassis_ctrl_hold_yaw(MAIN_POS_NAV_HOLD_YAW_DEG);
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_POINT_NAV)
    /* 车头朝上=yaw0°, 暖机期间保持当前航向不动. */
    chassis_ctrl_hold_yaw(0.0f);
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_SOKO_SELFTEST)
    /* 推箱求解自测: 摄像头 OpenART 通过 UART4 发来地图帧 → app_link 解析 →
     * chassis_menu_render_100ms() 每 100ms 自动在 IPS200 屏幕上渲染彩色方块.
     * 车不驱动, 无串口 printf 输出. */
    chassis_ctrl_stop();
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_SOLVE_VERIFY)
    /* 解算验证: 收摄像头地图 → 求解 → 虚拟走完比赛流程 (WAIT_MAP→SOLVE→
     * WAIT_LEAVE→WARMUP→PUSH_BOXES→RETURN_HOME→DONE), 全程车不动.
     * 地图色块由菜单渲染, 解算进度由 main_mode5_render_100ms() 叠加. */
    printf("SV_BOOT wait MAP from OpenART (UART4)...\n");
    chassis_ctrl_stop();
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_STATIC_VERIFY)
    printf("M6_BOOT static map solver screen verify...\n");
    chassis_ctrl_stop();
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_OPENART2_TEST)
    printf("OA2_BOOT wait BOX_CLASS from OpenART2 (UART1)...\n");
    chassis_ctrl_stop();
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_LEVEL1_TEST)
    /* 第一关测试模式:
     *   等待 OpenART1 发来地图 → 冻结该地图 → 解算 → 固定发车 → 跑航点 → 回固定发车点.
     *   上电后车停在发车区不动, 直到收到地图且解算成功后才自动发车. */
    printf("L1_BOOT wait MAP from OpenART (UART4)...\n");
    chassis_ctrl_hold_yaw(0.0f);
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_LEVEL2_TEST)
    printf("L2_BOOT wait MAP UART4 + BOX_CLASS UART1...\n");
    chassis_ctrl_hold_yaw(0.0f);
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_HARDCODED_MAP)
    /* 硬编码地图模式:
     *   上电立即用编译期字面量地图解算 → 暖机 → 自动跑航点 → 回发车区.
     *   无需等待串口地图帧, 无需人工发车. */
    printf("HCM_BOOT solving hardcoded map...\n");
    chassis_ctrl_hold_yaw(0.0f);
    main_hcm_solve();   /* 同步解算, 结果写入 s_hcm_solve_ok */
#elif (MAIN_RUN_MODE == MAIN_RUN_MODE_GAME)
    /* 游戏模式: 设置发车区 -Y 航向基准, 等待状态机调度. */
    chassis_ctrl_hold_yaw(APP_GAME_LAUNCH_FACE_YAW_DEG);
#else
    /* 游戏模式: 设置初始航向基准, 等待状态机调度. */
    chassis_ctrl_hold_yaw(0.0f);
#endif

    // ------------------------------------------------------------------
    // 4. PIT 定时中断初始化
    //    CH0: 5ms 姿态采样  CH1: 20ms 底盘闭环
    //    菜单开启时 CH2: 10ms 按键扫描（渲染在主循环）
    // ------------------------------------------------------------------
    pit_ms_init(PIT_CH0, 5);
    pit_ms_init(PIT_CH1, 20);
#if (CHASSIS_MENU_ENABLE != 0)
    pit_ms_init(PIT_CH2, 10);
#endif

    /* 硬件看门狗: 主循环卡死时 2s 自动复位 */
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
        RTWDOG_Refresh(RTWDOG);  /* 喂狗: 主循环活着就每 5ms 刷新 */

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
            /* 航点描述: { x(格), y(格), yaw(°) }
             *   x/y = 网格坐标, 运行时自动换算为米
             *   yaw = 目标航向°: 0=东(+X), 90=北(+Y), ±180=西(-X), -90=南(-Y)
             *   可设任意值, 自动 snap 到最近 90° 整数倍 */
            typedef struct { float x; float y; float yaw; } s_nav_wp_t;

            /* ═══════════ 航点序列 — 改这里 ═══════════ */
            static const s_nav_wp_t s_wps[] = {
                /*    x格    y格    yaw°   说明 */
                {   1.0f, 10.0f,  0.0f },  /* ① (1,1)→(1,10) 北↑ */
                {  14.0f, 10.0f,  0.0f },  /* ② (1,10)→(14,10) 东→ */
                {  14.0f,  1.0f,  0.0f },  /* ③ (14,10)→(14,1) 南↓ */
                {   1.0f,  1.0f,  0.0f },  /* ④ (14,1)→(1,1) 西← */
                
                {   1.0f, 10.0f,  0.0f },  /* ① (1,1)→(1,10) 北↑ */
                {  14.0f, 10.0f,  0.0f },  /* ② (1,10)→(14,10) 东→ */
                {  14.0f,  1.0f,  0.0f },  /* ③ (14,10)→(14,1) 南↓ */
                {   1.0f,  1.0f,  0.0f }, 

                {   1.0f, 10.0f,  0.0f },  /* ① (1,1)→(1,10) 北↑ */
                {  14.0f, 10.0f,  0.0f },  /* ② (1,10)→(14,10) 东→ */
                {  14.0f,  1.0f,  0.0f },  /* ③ (14,10)→(14,1) 南↓ */
                {   1.0f,  1.0f,  0.0f },  

                {   1.0f, 10.0f,  0.0f },  /* ① (1,1)→(1,10) 北↑ */
                {  14.0f, 10.0f,  0.0f },  /* ② (1,10)→(14,10) 东→ */
                {  14.0f,  1.0f,  0.0f },  /* ③ (14,10)→(14,1) 南↓ */
                {   1.0f,  1.0f,  0.0f },  /* 
                
                //  { S_NAV_MOVE_M,  7.0f, 10.0f  },
                // { S_NAV_MOVE_M,  1.0f, 10.0f },  /* ③ 右下角 (14,10) */
                // { S_NAV_MOVE_M,  9.0f, 10.0f },  /* ④ 左下角 (1,10)  */
                // { S_NAV_MOVE_M,  9.0f,  4.0f  },  /* ⑤ 中心  (7,5)    */
                // { S_NAV_MOVE_M,  9.0f,  10.0f  },  /* ⑥ 左上角 (1,1) 再次经过 */
                // { S_NAV_MOVE_M,  14.0f, 10.0f },  /* ⑦ 右下角 (14,10) 再次经过 */
                // { S_NAV_MOVE_M,  14.0f,  5.50f  },  /* ⑧ 左上角 (1,1)  作为回程中转 */
                // { S_NAV_MOVE_M,   14.0f,  1.0f  },  /* ⑨ 回起点 (1,5.5) */
                // { S_NAV_MOVE_M,  14.0f,  5.50f  },  /* ① 左上角 (1,1)   */
                // { S_NAV_MOVE_M,  8.0f, 5.50f  },  /* ② 右上角 (14,1)  */
                // { S_NAV_MOVE_M,  8.0f, 10.0f }, 
                // { S_NAV_MOVE_M,  1.0f, 10.0f }, 
                // { S_NAV_MOVE_M,  1.0f,  5.50f  },
                // { S_NAV_MOVE_M,  1.0f, 1.0f },
                // { S_NAV_MOVE_M,  1.0f,  5.50f  }, 
                

            }; 
                // { S_NAV_MOVE_M,  14.0f,  5.50f  },  /* ① 左上角 (1,1)   */
                // { S_NAV_MOVE_M,  14.0f, 10.0f  },  /* ② 右上角 (14,1)  */
                // { S_NAV_MOVE_M,  1.0f, 10.0f },  /* ③ 右下角 (14,10) */
                // { S_NAV_MOVE_M,  9.0f, 5.0f },  /* ④ 左下角 (1,10)  */
                // { S_NAV_MOVE_M,  6.0f,  4.0f  },  /* ⑤ 中心  (7,5)    */
                // { S_NAV_MOVE_M,  9.0f,  4.0f  },  /* ⑥ 左上角 (1,1) 再次经过 */
                // { S_NAV_MOVE_M,  14.0f, 10.0f },  /* ⑦ 右下角 (14,10) 再次经过 */
                // { S_NAV_MOVE_M,  1.0f,  1.0f  },  /* ⑧ 左上角 (1,1)  作为回程中转 */
                // { S_NAV_MOVE_M,   14.0f,  10.0f  },  /* ⑨ 回起点 (1,5.5) */
                // { S_NAV_MOVE_M,  14.0f,  5.50f  },  /* ① 左上角 (1,1)   */
                // { S_NAV_MOVE_M,  8.0f, 7.0f  },  /* ② 右上角 (14,1)  */
                // { S_NAV_MOVE_M,  9.0f, 7.0f },  /* ③ 右下角 (14,10) */
                // { S_NAV_MOVE_M,  9.0f, 5.0f },  /* ④ 左下角 (1,10)  */
                // { S_NAV_MOVE_M,  6.0f,  4.0f  },  /* ⑤ 中心  (7,5)    */
                // { S_NAV_MOVE_M,  9.0f,  4.0f  },  /* ⑥ 左上角 (1,1) 再次经过 */
                // { S_NAV_MOVE_M,  14.0f, 10.0f },  /* ⑦ 右下角 (14,10) 再次经过 */
                // { S_NAV_MOVE_M,  1.0f,  1.0f  },  /* ⑧ 左上角 (1,1)  作为回程中转 */
                // { S_NAV_MOVE_M,   14.0f,  10.0f  },  /* ⑨ 回起点 (1,5.5) */
                // { S_NAV_MOVE_M,  14.0f,  5.50f  },  /* ① 左上角 (1,1)   */
                // { S_NAV_MOVE_M,  8.0f, 7.0f  },  /* ② 右上角 (14,1)  */
                // { S_NAV_MOVE_M,  9.0f, 7.0f },  /* ③ 右下角 (14,10) */
                // { S_NAV_MOVE_M,  1.0f, 5.50f },  /* ④ 左下角 (1,10)  */
                // { S_NAV_MOVE_M,  6.0f,  4.0f  },  /* ⑤ 中心  (7,5)    */
                // { S_NAV_MOVE_M,  9.0f,  4.0f  },  /* ⑥ 左上角 (1,1) 再次经过 */
                // { S_NAV_MOVE_M,  14.0f, 10.0f },  /* ⑦ 右下角 (14,10) 再次经过 */
                // { S_NAV_MOVE_M,  1.0f,  1.0f  },  /* ⑧ 左上角 (1,1)  作为回程中转 */
                // { S_NAV_MOVE_M,   14.0f,  10.0f  },  /* ⑨ 回起点 (1,5.5) */
                // { S_NAV_MOVE_M,  14.0f,  5.50f  },  /* ① 左上角 (1,1)   */
                // { S_NAV_MOVE_M,  8.0f, 7.0f  },  /* ② 右上角 (14,1)  */
                // { S_NAV_MOVE_M,  9.0f, 7.0f },  /* ③ 右下角 (14,10) */
                // { S_NAV_MOVE_M,  9.0f, 5.0f },  /* ④ 左下角 (1,10)  */
                // { S_NAV_MOVE_M,  6.0f,  4.0f  },  /* ⑤ 中心  (7,5)    */
                // { S_NAV_MOVE_M,  9.0f,  4.0f  },  /* ⑥ 左上角 (1,1) 再次经过 */
                // { S_NAV_MOVE_M,  14.0f, 10.0f },  /* ⑦ 右下角 (14,10) 再次经过 */
                // { S_NAV_MOVE_M,  1.0f,  1.0f  },  /* ⑧ 左上角 (1,1)  作为回程中转 */
                // { S_NAV_MOVE_M,   14.0f,  10.0f  },  /* ⑨ 回起点 (1,5.5) */
                // { S_NAV_MOVE_M,  8.0f, 7.0f  },  /* ② 右上角 (14,1)  */
                // { S_NAV_MOVE_M,  9.0f, 7.0f },  /* ③ 右下角 (14,10) */
                // { S_NAV_MOVE_M,  9.0f, 5.0f },  /* ④ 左下角 (1,10)  */
                // { S_NAV_MOVE_M,  6.0f,  4.0f  },  /* ⑤ 中心  (7,5)    */
                // { S_NAV_MOVE_M,  9.0f,  4.0f  },  /* ⑥ 左上角 (1,1) 再次经过 */
                // { S_NAV_MOVE_M,  14.0f, 10.0f },  /* ⑦ 右下角 (14,10) 再次经过 */
                // { S_NAV_MOVE_M,  1.0f,  1.0f  },  /* ⑧ 左上角 (1,1)  作为回程中转 */
                // { S_NAV_MOVE_M,   14.0f,  10.0f  },  /* ⑨ 回起点 (1,5.5) */
                // { S_NAV_MOVE_M,  14.0f,  5.50f  },  /* ① 左上角 (1,1)   */
                // { S_NAV_MOVE_M,  8.0f, 7.0f  },  /* ② 右上角 (14,1)  */
                // { S_NAV_MOVE_M,  9.0f, 7.0f },  /* ③ 右下角 (14,10) */
                // { S_NAV_MOVE_M,  9.0f, 5.0f },  /* ④ 左下角 (1,10)  */
                // { S_NAV_MOVE_M,  6.0f,  4.0f  },  /* ⑤ 中心  (7,5)    */
                // { S_NAV_MOVE_M,  9.0f,  4.0f  },  /* ⑥ 左上角 (1,1) 再次经过 */
                // { S_NAV_MOVE_M,  14.0f, 10.0f },  /* ⑦ 右下角 (14,10) 再次经过 */
                // { S_NAV_MOVE_M,  1.0f,  1.0f  },  /* ⑧ 左上角 (1,1)  作为回程中转 */
                // { S_NAV_MOVE_M,   1.0f,  5.5f  },  /* ⑨ 回起点 (1,5.5) */
                
          //  };

            static uint8  s_wp_idx       = 0U;   /* 当前航点索引 (0 起, 到 count 停止) */
            static uint16 s_warmup_ticks = 0U;   /* 暖机计数 (5ms tick) */
            static uint8  s_nav_started  = 0U;   /* 0=暖机中, 1=遍历进行中 */


/* P0-修复 2026-06-30: 坐标系 +X=左,+Y=上,CCW=yaw正.
 *   车头朝上=yaw90°, 旋转矩阵已对齐, user_yaw 直接可用. */
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
                    printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%lu\n",
                           pose.x_m, pose.y_m,
                           tgt_x, tgt_y,
                           dist_sq,
                           pose.yaw_deg,
                           (int)chassis_ctrl_is_arrived(),
                           (int)s_wp_idx,
                           (unsigned long)g_chassis_arrival_count);
                    /* 诊断: 车体期望/实际速度 + 四轮期望/实际速度
                     * VOFA 模式下注释掉, 否则带前缀的 printf 会污染纯数据流 */
                    // {
                    //     float wfb[4];
                    //     chassis_ctrl_get_wheel_feedback_snapshot(wfb);
                    //     printf("D:vt=%.3f,%.3f vf=%.3f,%.3f | ",
                    //            g_chassis_diag_body_spd_tgt_vx,
                    //            g_chassis_diag_body_spd_tgt_vy,
                    //            g_chassis_diag_body_spd_fb_vx,
                    //            g_chassis_diag_body_spd_fb_vy);
                    //     printf("wt=%.3f,%.3f,%.3f,%.3f wf=%.3f,%.3f,%.3f,%.3f\n",
                    //            g_chassis_diag_wheel_tgt[0],
                    //            g_chassis_diag_wheel_tgt[1],
                    //            g_chassis_diag_wheel_tgt[2],
                    //            g_chassis_diag_wheel_tgt[3],
                    //            wfb[0], wfb[1], wfb[2], wfb[3]);
                    // }
                }
            }
        }
    #elif (MAIN_RUN_MODE == MAIN_RUN_MODE_SOKO_SELFTEST)
        /* 摄像头帧 → app_link ISR → 菜单自动渲染彩色方块, 主循环无需额外工作 */
    #elif (MAIN_RUN_MODE == MAIN_RUN_MODE_SOLVE_VERIFY)
        main_run_solve_verify_5ms();
    #elif (MAIN_RUN_MODE == MAIN_RUN_MODE_STATIC_VERIFY)
        main_run_static_verify_5ms();
    #elif (MAIN_RUN_MODE == MAIN_RUN_MODE_OPENART2_TEST)
        main_run_openart2_test_5ms();
    #elif (MAIN_RUN_MODE == MAIN_RUN_MODE_LEVEL1_TEST)
        main_run_level1_test_5ms();
        main_run_level1_test_log_50ms();
    #elif (MAIN_RUN_MODE == MAIN_RUN_MODE_LEVEL2_TEST)
        main_run_level2_test_5ms();
        main_run_level2_test_log_50ms();
    #elif (MAIN_RUN_MODE == MAIN_RUN_MODE_HARDCODED_MAP)
        main_run_hardcoded_map_5ms();
        main_run_hardcoded_map_log_50ms();
    #else
        Game_Logic_Task_Run();          /* 推箱子状态机 (非阻塞) */
    #endif

#if (CHASSIS_MENU_ENABLE != 0)
        /* 菜单渲染放到主循环，避免在 PIT 中断内刷屏造成控制节拍抖动。 */
        menu_render_div++;
        if (menu_render_div >= MAIN_MENU_RENDER_DIV)
        {
            menu_render_div = 0U;
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_OPENART2_TEST)
            main_openart2_test_render_100ms();
#else
#if (MAIN_RUN_MODE != MAIN_RUN_MODE_POINT_NAV)
            chassis_menu_render_100ms();
#endif
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_STATIC_VERIFY)
            main_mode6_render_100ms();
#endif
#if (MAIN_RUN_MODE == MAIN_RUN_MODE_SOLVE_VERIFY)
            main_mode5_render_100ms();   /* 叠加解算进度文字到菜单下方 */
#endif
#endif
        }
#endif

        /* P0-5: 节拍由 wait_for_tick() 在循环顶部统一接管, 此处不再 system_delay_ms */
    }
}
