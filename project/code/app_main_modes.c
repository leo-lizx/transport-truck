/*===========================================================================
 * [app_main_modes.c] 主程序运行模式实现
 *
 * 负责把 main loop 的 5ms tick 分发给不同模式：
 *   - 推箱完整流程
 *   - yaw 保持 / 单轮 PID / 点导航调试
 *   - 推箱求解器自测
 *   - OpenART1 链路测试
 *   - 静态地图自动推箱执行
 *
 * 所有业务模式都会参与编译，main.c 通过运行时配置选择当前模式。
 *===========================================================================*/

#include "app_main_modes.h"
#include "app_main_config.h"
#include "app_main_runtime.h"
#include "algo_sokoban_solver.h"
#include "app_game_logic.h"
#include "app_link.h"
#include "app_static_map_drive_helper.h"
#include "app_vision_fusion.h"
#include "chassis_ctrl.h"
#include "chassis_pid.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static app_main_options_t s_app_options = {
    APP_RUN_MODE_STATIC_MAP_DRIVE,
    APP_STATIC_MAP_SOURCE_MANUAL,
    (uint8)CHASSIS_WHEEL_LF,
    1U,
    3.3f,
    40.0f,
    23.0f,
    0.0f,
    0.0f
};

static uint8 app_main_is_link_mode(app_main_run_mode_e mode)
{
    return ((mode == APP_RUN_MODE_GAME) ||
            (mode == APP_RUN_MODE_OPENART1_TEST)) ? 1U : 0U;
}

static uint8 app_main_mode_is_valid(app_main_run_mode_e mode)
{
    return (mode <= APP_RUN_MODE_STATIC_MAP_DRIVE) ? 1U : 0U;
}

static uint8 app_main_static_map_source_is_valid(app_static_map_source_e source)
{
    return (source <= APP_STATIC_MAP_SOURCE_MANUAL) ? 1U : 0U;
}

/* 推箱算法自测地图：只在 SOKO_SELFTEST 模式编译，用于脱离视觉和底盘验证求解器。 */
static const char s_soko_selftest_map[MAP_ROWS][MAP_COLS + 1] = {
    "################",
    "#-#------------#",
    "#-.------#####-#",
    "##$###---#---#-#",
    "#----#---#.#-#-#",
    "#----#####.#-#-#",
    "#-------$--$-#-#",
    "#-----------##-#",
    "#--------------#",
    "#-----####-----#",
    "#--------------#",
    "################",
};

/*
 * 函数: app_main_selftest_action_to_char
 * 功能: 将推箱求解器输出的动作枚举转换成串口日志中的单字符。
 * 参数: act - SokoAction_e 动作枚举。
 * 返回: 'U'/'D'/'L'/'R' 表示上下左右；未知值返回 '?'。
 * 说明: 只用于 SOKO_SELFTEST 日志，不参与真实底盘控制。
 */
static char app_main_selftest_action_to_char(SokoAction_e act)
{
    /* 将枚举动作压缩成单字符，便于串口日志和 PC 端校验脚本读取。 */
    switch (act)
    {
    case SOKO_ACT_UP:    return 'U';
    case SOKO_ACT_DOWN:  return 'D';
    case SOKO_ACT_LEFT:  return 'L';
    case SOKO_ACT_RIGHT: return 'R';
    default:             return '?';
    }
}

/*
 * 函数: app_main_selftest_log
 * 功能: SOKO_SELFTEST 模式的统一日志输出。
 * 参数: fmt/... - printf 风格格式化字符串和参数。
 * 返回: 无。
 * 嵌入式注意:
 *   1. 使用固定长度栈缓冲区，避免动态分配。
 *   2. 输出会同时走 printf 和 UART_1，日志过密会占用串口带宽。
 *   3. 该函数在主循环 5ms 任务中调用，不应放入中断。
 */
static void app_main_selftest_log(const char *fmt, ...)
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

    /* 同时输出到 printf 和 UART_1，兼顾调试器控制台与外部串口工具。 */
    printf("%s", buffer);

    if (len > (int)(sizeof(buffer) - 1U))
    {
        len = (int)(sizeof(buffer) - 1U);
    }
    uart_write_buffer(UART_1, (const uint8 *)buffer, (uint32)len);
}

/*
 * 函数: app_main_selftest_print_map_and_player
 * 功能: 打印自测地图尺寸、玩家起点和完整 ASCII 地图。
 * 参数: player_pos - 推箱求解器使用的玩家起始格坐标。
 * 返回: 无。
 * 用途: 方便 PC 端脚本或人工对照“输入地图”和“求解结果”是否一致。
 */
static void app_main_selftest_print_map_and_player(Point_t player_pos)
{
    uint8 r;

    app_main_selftest_log("MAP_ROWS=%d\n", (int)MAP_ROWS);
    app_main_selftest_log("MAP_COLS=%d\n", (int)MAP_COLS);
    app_main_selftest_log("PLAYER_START=%d,%d\n", (int)player_pos.x, (int)player_pos.y);
    app_main_selftest_log("MAP_BEGIN\n");
    for (r = 0U; r < MAP_ROWS; r++)
    {
        app_main_selftest_log("%s\n", s_soko_selftest_map[r]);
    }
    app_main_selftest_log("MAP_END\n");
}

/*
 * 函数: app_main_soko_selftest_once
 * 功能: 执行一次离线推箱求解器自测，并打印每段动作序列。
 * 参数: 无。
 * 返回: 无。
 * 流程:
 *   1. 将内置 ASCII 地图转换为 MAP_* 枚举地图。
 *   2. 按 MAIN_POS_NAV_START_* 指定起点调用 Sokoban_Solve_Stage1。
 *   3. 将每个箱子的动作段输出为 U/D/L/R 字符串。
 * 说明: 该函数只验证算法，不会向底盘下发任何移动命令。
 */
static void app_main_soko_selftest_once(void)
{
    uint8 map[MAP_ROWS][MAP_COLS];
    Point_t player_pos;
    SokoFullSolution_t solution;
    uint8 r, c;

    player_pos.x = (int8)MAIN_POS_NAV_START_X_GRID;
    player_pos.y = (int8)MAIN_POS_NAV_START_Y_GRID;

    /* 复用 helper 公共接口，避免重复实现字符转枚举循环。 */
    App_StaticMapDrive_LoadCharMap(s_soko_selftest_map, map);

    app_main_selftest_log("SOKO_MAP=T4\n");
    app_main_selftest_print_map_and_player(player_pos);

    memset(&solution, 0, sizeof(solution));
    if (!Sokoban_Solve_Stage1(map, player_pos, &solution))
    {
        app_main_selftest_log("SOLVED=0\n");
        app_main_selftest_log("ERR=NO_SOLUTION\n");
        return;
    }

    app_main_selftest_log("SOLVED=%d\n", (int)solution.is_solved);
    app_main_selftest_log("BOXES=%d\n", (int)solution.total_boxes);
    for (r = 0U; r < solution.total_boxes; r++)
    {
        uint16 i;
        app_main_selftest_log("SEG%d_COUNT=%d\n", (int)r, (int)solution.sub_solutions[r].count);
        app_main_selftest_log("SEG%d_PATH=", (int)r);
        for (i = 0U; i < solution.sub_solutions[r].count; i++)
        {
            app_main_selftest_log("%c", app_main_selftest_action_to_char(solution.sub_solutions[r].actions[i]));
        }
        app_main_selftest_log("\n");
        app_main_selftest_log("PLAYER_END=%d,%d\n",
                              (int)solution.player_end_pos[r].x,
                              (int)solution.player_end_pos[r].y);
    }
}

/*
 * 函数: app_main_soko_selftest_periodic_5ms
 * 功能: SOKO_SELFTEST 模式的 5ms 周期任务。
 * 参数: 无。
 * 返回: 无。
 * 调度: App_MainModes_Task5ms 每 5ms 调用一次，本函数内部 200 分频后约 1s 跑一次自测。
 */
static void app_main_soko_selftest_periodic_5ms(void)
{
    static uint16 s_div = 0U;

    /* 5ms 调度下每 200 次约 1s 打印一次，避免日志过密。 */
    s_div++;
    if (s_div >= 200U)
    {
        s_div = 0U;
        app_main_selftest_log("===SOKO_SELFTEST_ALIVE===\n");
        app_main_soko_selftest_once();
    }
}

typedef enum {
    /* 等待地图来源准备好：手写地图或 OpenART1 MAP 帧。 */
    SMD_PHASE_WAIT_MAP = 0,
    /* 地图锁定后给底盘和视觉融合一点稳定时间。 */
    SMD_PHASE_WARMUP,
    /* 按推箱求解结果逐段执行航点。 */
    SMD_PHASE_PUSH_BOXES,
    /* 所有箱子处理完后返回发车点。 */
    SMD_PHASE_RETURN_HOME,
    /* 任务结束，保持停车。 */
    SMD_PHASE_DONE
} smd_phase_e;

/* STATIC_MAP_DRIVE 模式的跨 tick 状态。
 * 全部只在主循环 5ms 任务中修改，不从中断直接写入。
 */
static SokoFullSolution_t  s_smd_solution;
static SokoWaypointPath_t  s_smd_waypoints;
static uint8               s_smd_map[MAP_ROWS][MAP_COLS];
static uint8               s_smd_solve_ok     = 0U;
static uint8               s_smd_sub_idx      = 0U;
static uint16              s_smd_wp_idx       = 0U;
static uint8               s_smd_navigating   = 0U;
static smd_phase_e         s_smd_phase        = SMD_PHASE_WAIT_MAP;
static uint16              s_smd_warmup_ticks = 0U;
static uint32              s_smd_map_recv_ms  = 0U;

static float app_main_smd_grid_x_to_abs_m(float grid_x)
{
    return MAIN_POS_GRID_TO_M_X(grid_x);
}

static float app_main_smd_grid_y_to_abs_m(float grid_y)
{
    return MAIN_POS_GRID_TO_M_Y(grid_y);
}

static void app_main_smd_grid_to_abs_m(Point_t grid, float *out_x_m, float *out_y_m)
{
    if (out_x_m != NULL) { *out_x_m = app_main_smd_grid_x_to_abs_m((float)grid.x); }
    if (out_y_m != NULL) { *out_y_m = app_main_smd_grid_y_to_abs_m((float)grid.y); }
}

static void app_main_smd_log_grid_target(const char *tag,
                                         uint8 sub_idx,
                                         uint16 wp_idx,
                                         Point_t grid,
                                         float abs_x_m,
                                         float abs_y_m)
{
    const float start_x_m = app_main_smd_grid_x_to_abs_m(MAIN_POS_NAV_START_X_GRID);
    const float start_y_m = app_main_smd_grid_y_to_abs_m(MAIN_POS_NAV_START_Y_GRID);

    printf("SMD_%s sub=%u idx=%u grid=%d,%d abs_m=%.4f,%.4f d_start=%.4f,%.4f\n",
           tag,
           (unsigned)sub_idx,
           (unsigned)wp_idx,
           (int)grid.x,
           (int)grid.y,
           (double)abs_x_m,
           (double)abs_y_m,
           (double)(abs_x_m - start_x_m),
           (double)(abs_y_m - start_y_m));
}

static void app_main_smd_dispatch_waypoint(Point_t grid)
{
    float target_x_m;
    float target_y_m;

    app_main_smd_grid_to_abs_m(grid, &target_x_m, &target_y_m);
    app_main_smd_log_grid_target("WP", s_smd_sub_idx, s_smd_wp_idx, grid, target_x_m, target_y_m);

    /* 静态地图航点是算法网格中心；下发给底盘的是对应的绝对米制坐标。
     * 日志里的 d_start 用来核对相对 (1, 5.5) 发车点的位移，包含首段半格 Y 偏移。 */
    chassis_ctrl_move_to_m(target_x_m, target_y_m, 0.0f);
}

static void app_main_smd_print_waypoint_path(const SokoWaypointPath_t *path)
{
    uint16 i;

    if (path == NULL)
    {
        return;
    }

    printf("SMD_PATH_BEGIN count=%u\n", (unsigned)path->count);
    for (i = 0U; i < path->count; i++)
    {
        float x_m;
        float y_m;
        app_main_smd_grid_to_abs_m(path->points[i], &x_m, &y_m);
        printf("SMD_PATH[%u]=grid:%d,%d abs_m:%.4f,%.4f\n",
               (unsigned)i,
               (int)path->points[i].x,
               (int)path->points[i].y,
               (double)x_m,
               (double)y_m);
    }
    printf("SMD_PATH_END\n");
}

/* 手写静态地图：'#'=墙, '-'=空地, '.'=目标, '$'=箱子, '*'=炸弹。 */
static const char s_smd_manual_map[MAP_ROWS][MAP_COLS + 1] = {
    "################",
    "#-#.-----------#",
    "#-#------###---#",
    "####-#.-----#--#",
    "#----#--#-#-#--#",
    "#-----###-#----#",
    "#--$#-----$-#.-#",
    "#-----$----##--#",
    "#----#---------#",
    "#--------------#",
    "#----#---------#",
    "################",
};

/*
 * 函数: app_main_smd_solve_after_recv
 * 功能: 地图准备好后直接调用 Stage1 求解器和航点转换接口。
 * 参数: 无，输入来自模块静态变量 s_smd_map。
 * 返回: 无，求解成功与否写入 s_smd_solve_ok。
 * 嵌入式注意: 求解只在地图锁定时执行一次，不放在每个 5ms tick 内重复执行。
 */
static void app_main_smd_solve_after_recv(void)
{
    Point_t player;
    uint8   r, b;

    /* 打印最终参与求解的地图，便于确认地图内容是否正确。 */
    printf("SMD_MAP_BEGIN\n");
    for (r = 0U; r < MAP_ROWS; r++)
    {
        char line[MAP_COLS + 1];
        uint8 c;
        for (c = 0U; c < MAP_COLS; c++)
        {
            line[c] = App_StaticMapDrive_MapToChar(s_smd_map[r][c]);
        }
        line[MAP_COLS] = '\0';
        printf("%s\n", line);
    }
    printf("SMD_MAP_END\n");

    /* 发车格取整数坐标，与底盘里程计零点对齐。 */
    player.x = (int8)CHASSIS_START_GRID_X;
    player.y = (int8)CHASSIS_START_GRID_Y;
    printf("SMD_PLAYER_START=%d,%d\n", (int)player.x, (int)player.y);

    s_smd_solve_ok = 0U;
    memset(&s_smd_solution, 0, sizeof(s_smd_solution));

    /* Stage1：贪心分配 + 单箱 BFS，一次求出所有箱子的动作序列。 */
    if (!Sokoban_Solve_Stage1(s_smd_map, player, &s_smd_solution))
    {
        printf("SMD_ERR=NO_SOLUTION\n");
        return;
    }
    if (!s_smd_solution.is_solved || s_smd_solution.total_boxes == 0U)
    {
        printf("SMD_ERR=UNSOLVED boxes=%d\n", (int)s_smd_solution.total_boxes);
        return;
    }

    /* 每段动作序列转换为转弯航点，鼓续追加到统一航点数组。 */
    memset(&s_smd_waypoints, 0, sizeof(s_smd_waypoints));
    {
        Point_t cur_pos = player;
        for (b = 0U; b < s_smd_solution.total_boxes; b++)
        {
            SokoWaypointPath_t seg;
            uint16 i;
            /* 同向连续动作压缩为一个转弯点，减少底盘下发频次。 */
            Sokoban_Actions_To_Waypoints(
                s_smd_solution.sub_solutions[b].actions,
                s_smd_solution.sub_solutions[b].count,
                cur_pos, &seg);
            for (i = 0U; i < seg.count; i++)
            {
                if (s_smd_waypoints.count < SOKOBAN_MAX_WAYPOINTS)
                {
                    s_smd_waypoints.points[s_smd_waypoints.count] = seg.points[i];
                    s_smd_waypoints.count++;
                }
            }
            /* 本段结束后玩家坐标由求解器给出，作为下一段起点。 */
            cur_pos = s_smd_solution.player_end_pos[b];
        }
    }

    s_smd_sub_idx    = 0U;
    s_smd_wp_idx     = 0U;
    s_smd_navigating = 0U;
    s_smd_solve_ok   = 1U;

    printf("SMD_SOLVED=1 boxes=%d path_wp=%d\n",
           (int)s_smd_solution.total_boxes,
           (int)s_smd_waypoints.count);
    app_main_smd_print_waypoint_path(&s_smd_waypoints);
}

#define MAIN_SMD_LINK_LOSS_MS (200U)

/*
 * 函数: app_main_smd_link_alive
 * 功能: 判断 OpenART1 链路在 STATIC_MAP_DRIVE 模式下是否仍可用。
 * 参数: 无。
 * 返回: 1=最近 MAIN_SMD_LINK_LOSS_MS 内有心跳或合法帧；0=链路静默/从未在线。
 * 说明: 使用 app_link_get_ms() 与 g_link_last_hb_ms 的同源时基，避免不同计时源造成误判。
 */
static uint8 app_main_smd_link_alive(void)
{
    /* STATIC_MAP_DRIVE 中视觉链路只用于融合和到站 snap，掉线不直接重算地图。 */
    uint32 last_ok = g_link_last_hb_ms;
    uint32 now_ms = app_link_get_ms();
    uint32 silence_ms = (now_ms >= last_ok) ? (now_ms - last_ok) : 0U;
    return ((last_ok != 0U) && (silence_ms <= (uint32)MAIN_SMD_LINK_LOSS_MS)) ? 1U : 0U;
}

/*
 * 函数: app_main_smd_vision_task_5ms
 * 功能: 驱动视觉融合和一致性监控的 5ms 入口。
 * 参数: 无。
 * 返回: 无。
 * 策略: 任务完成或链路掉线时禁止融合，防止旧观测把里程计拉到错误位置。
 */
static void app_main_smd_vision_task_5ms(void)
{
    /* 任务未完成且链路在线时才允许视觉融合，避免用陈旧观测修正里程计。 */
    uint8 allow = (uint8)((s_smd_phase != SMD_PHASE_DONE) && (app_main_smd_link_alive() != 0U));
    app_vision_fusion_task(allow);
    app_vision_fusion_consistency_tick(allow);
}

/*
 * 函数: app_main_smd_nav_arrived_for_waypoint
 * 功能: 判断当前航点是否可以认为“到达”。
 * 参数: 无。
 * 返回: 1=可切换到下一航点；0=继续等待。
 * 判定层次:
 *   1. chassis_ctrl_is_arrived() 必须先满足里程计到站。
 *   2. 如果启用视觉到站 Snap 且链路在线，还要等待 Snap 完成/超时/拒绝。
 *   3. 链路掉线时直接按里程计放行，避免卡死在视觉确认阶段。
 */
static uint8 app_main_smd_nav_arrived_for_waypoint(void)
{
    if (chassis_ctrl_is_arrived() == 0U)
    {
        /* 还没到 odom 目标时取消未完成 snap，防止上一航点的 snap 影响下一次判断。 */
        app_vision_fusion_snap_cancel();
        return 0U;
    }

#if CHASSIS_VISION_SNAP_ON_ARRIVE_ENABLE
    if (app_main_smd_link_alive() == 0U)
    {
        /* 视觉掉线时放行 odom 到站结果，避免任务在 snap 阶段卡死。 */
        app_vision_fusion_snap_cancel();
        return 1U;
    }
    {
        float tx_m = 0.0f;
        float ty_m = 0.0f;
        app_vision_snap_state_e st;
        chassis_ctrl_get_point_nav_target_m(&tx_m, &ty_m);
        app_vision_fusion_snap_request(tx_m, ty_m);
        st = app_vision_fusion_snap_state();
        if ((st == APP_VISION_SNAP_DONE)    ||
            (st == APP_VISION_SNAP_TIMEOUT) ||
            (st == APP_VISION_SNAP_REJECT))
        {
            return 1U;
        }
        return 0U;
    }
#else
    return 1U;
#endif
}

/*
 * 函数: app_main_smd_wait_map_5ms
 * 功能: STATIC_MAP_DRIVE 的等待地图阶段处理。
 * 参数: 无。
 * 返回: 无。
 * 两种来源:
 *   - APP_STATIC_MAP_SOURCE_MANUAL: 加载本文件内置 ASCII 地图。
 *   - APP_STATIC_MAP_SOURCE_OPENART1: 等待 app_link 收到 OpenART1 MAP 帧后复制快照。
 * 状态变化: 求解成功后切换到 SMD_PHASE_WARMUP，并关闭 UART4 原始字节探针。
 */
static void app_main_smd_wait_map_5ms(void)
{
    static uint8 s_manual_done = 0U;
    static uint16 s_div = 0U;
    uint32 now_recv_ms;

    if (s_app_options.static_map_source == APP_STATIC_MAP_SOURCE_MANUAL)
    {
        if (s_manual_done == 0U)
        {
            s_manual_done = 1U;
            /* 手写地图只加载一次，随后立即进入求解和执行流程。 */
            App_StaticMapDrive_LoadCharMap(s_smd_manual_map, s_smd_map);
            printf("SMD_MAP_MANUAL loaded %dx%d\n", (int)MAP_ROWS, (int)MAP_COLS);
            app_main_smd_solve_after_recv();
            if (s_smd_solve_ok)
            {
                App_MainRuntime_DisableUart4Tap();
                printf("SMD_MAP_LOCKED rx_keep_on (manual)\n");
                s_smd_phase = SMD_PHASE_WARMUP;
                s_smd_warmup_ticks = 0U;
            }
        }
        return;
    }

    now_recv_ms = g_link_last_map_ms;
    if (now_recv_ms != 0U)
    {
        if (now_recv_ms != s_smd_map_recv_ms)
        {
            /* 收到新的合法 MAP 帧后复制 seq-lock 快照，再做一次性求解。 */
            s_smd_map_recv_ms = now_recv_ms;
            app_link_get_map_snapshot(s_smd_map);
            printf("SMD_MAP_RECV ms=%lu ok=%lu crc=%lu len=%lu\n",
                   (unsigned long)now_recv_ms,
                   (unsigned long)g_link_stats.frames_ok,
                   (unsigned long)g_link_stats.frames_crc_err,
                   (unsigned long)g_link_stats.frames_len_err);
            app_main_smd_solve_after_recv();
            if (s_smd_solve_ok)
            {
                App_MainRuntime_DisableUart4Tap();
                printf("SMD_MAP_LOCKED rx_keep_on\n");
                s_smd_phase = SMD_PHASE_WARMUP;
                s_smd_warmup_ticks = 0U;
            }
        }
        return;
    }

    if (++s_div >= 200U)
    {
        s_div = 0U;
        /* 等待地图时每约 1s 打印链路统计，帮助区分没接线/CRC 错/长度错。 */
        printf("SMD_WAIT_MAP hb=%lu ok=%lu crc=%lu len=%lu byte_to=%lu drop=%lu\n",
               (unsigned long)g_link_stats.hb_cnt,
               (unsigned long)g_link_stats.frames_ok,
               (unsigned long)g_link_stats.frames_crc_err,
               (unsigned long)g_link_stats.frames_len_err,
               (unsigned long)g_link_stats.frames_byte_timeout,
               (unsigned long)g_link_stats.sync_drops);
    }
}

/*
 * 函数: app_main_static_map_drive_5ms
 * 功能: STATIC_MAP_DRIVE 模式主状态机，每 5ms 调用一次。
 * 参数: 无。
 * 返回: 无。
 * 状态机:
 *   WAIT_MAP     : 等地图并求解。
 *   WARMUP       : 求解后短暂等待硬件稳定。
 *   PUSH_BOXES   : 按航点执行每个箱子的推箱路径。
 *   RETURN_HOME  : 所有箱子完成后回到发车点。
 *   DONE         : 停车并保持。
 * 控制原则: 每次只下发一个航点，等待到站确认后再推进索引，避免 5ms 周期内反复覆盖目标。
 */
static void app_main_static_map_drive_5ms(void)
{
    if (s_smd_phase == SMD_PHASE_WAIT_MAP)
    {
        app_main_smd_wait_map_5ms();
        return;
    }

    if (!s_smd_solve_ok || (s_smd_phase == SMD_PHASE_DONE))
    {
        return;
    }

    if (s_smd_phase == SMD_PHASE_WARMUP)
    {
        /* 求解完成后延迟启动导航，给底盘姿态、里程计、视觉链路留稳定窗口。 */
        if (++s_smd_warmup_ticks >= MAIN_POINT_NAV_WARMUP_TICKS)
        {
            s_smd_phase = SMD_PHASE_PUSH_BOXES;
        }
        return;
    }

    if (!s_smd_navigating)
    {
        Point_t next;
        if (s_smd_phase == SMD_PHASE_PUSH_BOXES)
        {
            if (s_smd_wp_idx >= s_smd_waypoints.count)
            {
                const float home_x_m = app_main_smd_grid_x_to_abs_m(MAIN_POS_NAV_START_X_GRID);
                const float home_y_m = app_main_smd_grid_y_to_abs_m(MAIN_POS_NAV_START_Y_GRID);
                s_smd_phase = SMD_PHASE_RETURN_HOME;
                printf("SMD_RETURN_HOME grid=%.2f,%.2f abs_m=%.4f,%.4f d_start=0.0000,0.0000\n",
                       (double)MAIN_POS_NAV_START_X_GRID,
                       (double)MAIN_POS_NAV_START_Y_GRID,
                       (double)home_x_m,
                       (double)home_y_m);
                chassis_ctrl_move_to_m(home_x_m, home_y_m, 0.0f);
                s_smd_navigating = 1U;
                return;
            }
            next = s_smd_waypoints.points[s_smd_wp_idx];
            /* 下发一个航点后等待到站确认，避免每个 tick 重复覆盖目标。 */
            app_main_smd_dispatch_waypoint(next);
            s_smd_navigating = 1U;
        }
        return;
    }

    if (!app_main_smd_nav_arrived_for_waypoint())
    {
        return;
    }

    s_smd_navigating = 0U;

    if (s_smd_phase == SMD_PHASE_PUSH_BOXES)
    {
        /* 到达当前航点后推进索引，下一轮循环再下发下一个航点。 */
        s_smd_wp_idx++;
    }
    else if (s_smd_phase == SMD_PHASE_RETURN_HOME)
    {
        s_smd_phase = SMD_PHASE_DONE;
        chassis_ctrl_stop();
        printf("SMD_DONE\n");
    }
}

/*
 * 函数: app_main_static_map_drive_log_50ms
 * 功能: 输出 STATIC_MAP_DRIVE 的运行状态 CSV 日志。
 * 参数: 无。
 * 返回: 无。
 * 调度: 由 5ms 任务内部分频到约 50ms 输出一次。
 * 字段: pose(x,y)、target(x,y)、距离平方、yaw、到站标志、阶段、箱子段索引、航点索引。
 */
static void app_main_static_map_drive_log_50ms(void)
{
    static uint8 s_div = 0U;
    if (++s_div < 10U) return;
    s_div = 0U;
    {
        /* CSV 风格运行日志：当前位置、目标点、距离平方、航向、阶段和航点索引。 */
        chassis_pose_t pose = chassis_ctrl_get_pose();
        float tgt_x = 0.0f, tgt_y = 0.0f, dx, dy, dist_sq;
        chassis_ctrl_get_point_nav_target_m(&tgt_x, &tgt_y);
        dx = tgt_x - pose.x_m;
        dy = tgt_y - pose.y_m;
        dist_sq = dx * dx + dy * dy;
        printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d\n",
               pose.x_m, pose.y_m,
               tgt_x, tgt_y,
               dist_sq,
               pose.yaw_deg,
               (int)chassis_ctrl_is_arrived(),
               (int)s_smd_phase,
               (int)s_smd_sub_idx,
               (int)s_smd_wp_idx);
    }
}

/*
 * 函数: app_main_apply_debug_wheel_pid
 * 功能: SINGLE_WHEEL 调试模式下临时覆盖指定轮子的 PID 参数。
 * 参数: 无，参数来自 main.c 传入的 app_main_options_t。
 * 返回: 无。
 * 说明: 只修改活动调参结构，不改默认值，便于快速试参。
 */
static void app_main_apply_debug_wheel_pid(void)
{
    uint8 idx = s_app_options.single_wheel_index;

    if (idx >= CHASSIS_CTRL_TUNE_WHEEL_COUNT)
    {
        idx = (uint8)CHASSIS_WHEEL_LF;
    }

    /* 单轮调试只覆盖活动参数，不改变默认配置来源。 */
    (void)chassis_ctrl_set_wheel_pid_tune(idx,
                                          s_app_options.single_wheel_pid_kp,
                                          s_app_options.single_wheel_pid_ki,
                                          s_app_options.single_wheel_pid_kd);
}

typedef struct
{
    /* act 决定 a/b 按网格解释还是按米制坐标解释。 */
    uint8 act;
    float a;
    float b;
} app_main_point_nav_wp_t;

#define APP_MAIN_NAV_MOVE_GRID  (0U)
#define APP_MAIN_NAV_MOVE_M     (2U)

static const app_main_point_nav_wp_t s_point_nav_wps[] = {
    { APP_MAIN_NAV_MOVE_GRID,  8.0f,  6.0f  },
    { APP_MAIN_NAV_MOVE_GRID,  8.0f,  7.0f  },
    { APP_MAIN_NAV_MOVE_GRID,  9.0f,  7.0f  },
    { APP_MAIN_NAV_MOVE_GRID,  9.0f,  5.0f  },
    { APP_MAIN_NAV_MOVE_GRID,  6.0f,  4.0f  },
    { APP_MAIN_NAV_MOVE_GRID,  9.0f,  4.0f  },
    { APP_MAIN_NAV_MOVE_GRID, 14.0f, 10.0f  },
    { APP_MAIN_NAV_MOVE_GRID,  1.0f,  1.0f  },
    { APP_MAIN_NAV_MOVE_M,     1.0f,  5.5f  },
};

/*
 * 函数: app_main_point_nav_dispatch
 * 功能: 将 POINT_NAV 航点描述转换为底盘控制命令。
 * 参数: wp - 航点配置，act 决定 a/b 是网格坐标还是米制坐标。
 * 返回: 无。
 * 说明: APP_MAIN_NAV_MOVE_GRID 直接下发网格；APP_MAIN_NAV_MOVE_M 当前仍通过 MAIN_POS_GRID_TO_M_* 转换。
 */
static void app_main_point_nav_dispatch(const app_main_point_nav_wp_t *wp)
{
    /* POINT_NAV 调试模式支持混合测试网格目标和米制目标。 */
    if (APP_MAIN_NAV_MOVE_GRID == wp->act)
    {
        chassis_ctrl_move_to_grid((uint8)wp->a, (uint8)wp->b);
    }
    else
    {
        chassis_ctrl_move_to_m(MAIN_POS_GRID_TO_M_X(wp->a),
                               MAIN_POS_GRID_TO_M_Y(wp->b),
                               0.0f);
    }
}

/*
 * 函数: app_main_point_nav_task_5ms
 * 功能: POINT_NAV 调试模式的航点序列执行器。
 * 参数: 无。
 * 返回: 无。
 * 流程:
 *   1. warmup 结束后下发第一个航点。
 *   2. chassis_ctrl_is_arrived() 为真时推进到下一个航点。
 *   3. 每约 50ms 打印一次位置误差，供串口绘图或调参。
 */
static void app_main_point_nav_task_5ms(void)
{
    static uint8  s_wp_idx       = 0U;
    static uint16 s_warmup_ticks = 0U;
    static uint8  s_nav_started  = 0U;

    if (!s_nav_started)
    {
        /* 上电后先 warmup，再下发第一点，避免传感器初值未稳定时就开始跑。 */
        if (++s_warmup_ticks >= MAIN_POINT_NAV_WARMUP_TICKS)
        {
            s_nav_started = 1U;
            app_main_point_nav_dispatch(&s_point_nav_wps[0]);
        }
    }
    else if (s_wp_idx < (uint8)(sizeof(s_point_nav_wps) / sizeof(s_point_nav_wps[0])))
    {
        if (chassis_ctrl_is_arrived())
        {
            s_wp_idx++;
            if (s_wp_idx < (uint8)(sizeof(s_point_nav_wps) / sizeof(s_point_nav_wps[0])))
            {
                app_main_point_nav_dispatch(&s_point_nav_wps[s_wp_idx]);
            }
        }
    }

    {
        static uint8 s_pos_div = 0U;
        if (++s_pos_div >= 10U)
        {
            /* 约 50ms 打印一次点导航误差，方便调参时画曲线。 */
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

/*
 * 函数: app_main_openart1_test_task_5ms
 * 功能: OpenART1 串口链路测试模式任务。
 * 参数: 无。
 * 返回: 无。
 * 调度: 5ms 调用，内部约 1s 输出一次链路统计。
 * 输出: 串口打印帧计数/CRC/长度错误/心跳/车格坐标，同时在 IPS 屏幕显示摘要。
 */
static void app_main_openart1_test_task_5ms(void)
{
    static uint16 s_link_print_div = 0U;
    s_link_print_div++;
    if (s_link_print_div >= 200U)
    {
        /* 每约 1s 同步输出串口日志和 IPS 屏幕统计，用于检查 OpenART1 帧质量。 */
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

void App_MainModes_Config(const app_main_options_t *options)
{
    app_main_options_t next = s_app_options;

    if (options != NULL)
    {
        next = *options;
    }

    if (app_main_mode_is_valid(next.run_mode) == 0U)
    {
        next.run_mode = APP_RUN_MODE_STATIC_MAP_DRIVE;
    }
    if (app_main_static_map_source_is_valid(next.static_map_source) == 0U)
    {
        next.static_map_source = APP_STATIC_MAP_SOURCE_MANUAL;
    }
    if (next.single_wheel_index >= CHASSIS_CTRL_TUNE_WHEEL_COUNT)
    {
        next.single_wheel_index = (uint8)CHASSIS_WHEEL_LF;
    }

    s_app_options = next;
}

/*
 * 函数: App_MainModes_InitCommunication
 * 功能: 初始化当前运行模式所需的通信外设。
 * 参数: 无。
 * 返回: 无。
 * 调用时机: main 初始化阶段，底盘控制启动前调用。
 * 嵌入式注意: 这里只做 UART 和协议层初始化，不启动耗时业务逻辑。
 */
void App_MainModes_InitCommunication(void)
{
    if ((app_main_is_link_mode(s_app_options.run_mode) != 0U) ||
        ((s_app_options.run_mode == APP_RUN_MODE_STATIC_MAP_DRIVE) &&
         (s_app_options.static_map_source == APP_STATIC_MAP_SOURCE_OPENART1)))
    {
        /* 需要视觉链路的模式统一初始化 OpenART1 UART 和协议解析器。 */
        uart_init(MAIN_OPENART1_UART, 115200, MAIN_OPENART1_UART_TX, MAIN_OPENART1_UART_RX);
        uart_rx_interrupt(MAIN_OPENART1_UART, 1);
        app_link_init();
    }

    if (s_app_options.run_mode == APP_RUN_MODE_OPENART1_TEST)
    {
        uart_init(MAIN_TEST_LOG_UART, 115200, MAIN_TEST_LOG_UART_TX, MAIN_TEST_LOG_UART_RX);
    }
}

/*
 * 函数: App_MainModes_AfterChassisInit
 * 功能: 底盘初始化完成后，为当前运行模式设置初始控制状态。
 * 参数: 无。
 * 返回: 无。
 * 典型动作: 启动单轮 PID 调试、保持 yaw、停车、清屏或打印启动信息。
 */
void App_MainModes_AfterChassisInit(void)
{
    switch (s_app_options.run_mode)
    {
    case APP_RUN_MODE_SINGLE_WHEEL:
        if (s_app_options.single_wheel_force_pid != 0U)
        {
            app_main_apply_debug_wheel_pid();
        }
        chassis_ctrl_start_single_wheel_pid_debug(s_app_options.single_wheel_index,
                                                  s_app_options.single_wheel_target_mps);
        break;

    case APP_RUN_MODE_YAW_HOLD:
        chassis_ctrl_hold_yaw(s_app_options.hold_yaw_target_deg);
        break;

    case APP_RUN_MODE_POINT_NAV:
        chassis_ctrl_hold_yaw(0.0f);
        break;

    case APP_RUN_MODE_SOKO_SELFTEST:
        uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);
        chassis_ctrl_stop();
        break;

    case APP_RUN_MODE_STATIC_MAP_DRIVE:
        /* UART1 在该模式主要作为 printf 输出口，关闭其 RX 中断避免无关输入干扰。 */
        uart_rx_interrupt(UART_1, 0);
        if (s_app_options.static_map_source == APP_STATIC_MAP_SOURCE_MANUAL)
        {
            printf("SMD_BOOT: TX=UART1(B12) MAP=MANUAL (hardcoded)\n");
        }
        else
        {
            App_MainRuntime_EnableUart4Tap();
            printf("SMD_BOOT: TX=UART1(B12) RX=UART4(C16) wait MAP frame...\n");
        }
        break;

    case APP_RUN_MODE_OPENART1_TEST:
        chassis_ctrl_stop();
        ips200_clear();
        ips200_show_string(0, 0, "OPENART1 UART4");
        ips200_show_string(0, 16, "WAIT RX...");
        break;

    case APP_RUN_MODE_GAME:
    default:
        chassis_ctrl_hold_yaw(0.0f);
        break;
    }
}

/*
 * 函数: App_MainModes_Task5ms
 * 功能: 主程序 5ms 周期分发入口。
 * 参数: 无。
 * 返回: 无。
 * 调用时机: main loop 每消费一个 PIT tick 后调用。
 * 说明: 通过运行时枚举选择业务分支，所有模式共享同一个固件镜像。
 */
void App_MainModes_Task5ms(void)
{
    switch (s_app_options.run_mode)
    {
    case APP_RUN_MODE_SINGLE_WHEEL:
        chassis_pid_debug_task_5ms();
        break;

    case APP_RUN_MODE_YAW_HOLD:
        chassis_ctrl_attitude_debug_task_5ms();
        break;

    case APP_RUN_MODE_POINT_NAV:
        app_main_point_nav_task_5ms();
        break;

    case APP_RUN_MODE_SOKO_SELFTEST:
        app_main_soko_selftest_periodic_5ms();
        break;

    case APP_RUN_MODE_STATIC_MAP_DRIVE:
        if (s_smd_phase == SMD_PHASE_WAIT_MAP)
        {
            /* 仅等待地图阶段保留 UART4 原始字节探针；地图锁定后由协议层和视觉融合接管。 */
            App_MainRuntime_DrainUart4Tap5ms();
        }
        app_main_smd_vision_task_5ms();
        app_main_static_map_drive_5ms();
        app_main_static_map_drive_log_50ms();
        break;

    case APP_RUN_MODE_OPENART1_TEST:
        app_main_openart1_test_task_5ms();
        break;

    case APP_RUN_MODE_GAME:
    default:
        Game_Logic_Task_Run();
        break;
    }
}

/*
 * 函数: App_MainModes_ShouldRenderMenu
 * 功能: 判断当前模式是否允许普通底盘菜单刷新。
 * 参数: 无。
 * 返回: 1=允许渲染菜单；0=当前模式独占屏幕。
 * 说明: OpenART1 测试模式会直接在 IPS 上显示链路统计，因此关闭普通菜单。
 */
uint8 App_MainModes_ShouldRenderMenu(void)
{
    return (s_app_options.run_mode != APP_RUN_MODE_OPENART1_TEST) ? 1U : 0U;
}

uint8 App_MainModes_IsStaticMapDrive(void)
{
    return (s_app_options.run_mode == APP_RUN_MODE_STATIC_MAP_DRIVE) ? 1U : 0U;
}
