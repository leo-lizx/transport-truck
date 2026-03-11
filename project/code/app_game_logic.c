#include "app_game_logic.h"

uint8 g_game_map[MAP_ROWS][MAP_COLS];
Point_t g_player_pos = {0, 0}; // 车模实时坐标

// 状态机内部使用的静态变量
static GameStage_e current_stage = STAGE_PENDING_SCOUT;
static uint8 is_navigating = 0;       // 是否正在移动的标志位
static ObservePoint_t current_target; // 当前需要前往的观察点

// 推箱子求解相关
static SokoFullSolution_t  g_soko_solution;
static SokoWaypointPath_t  g_soko_waypoints;
static uint8  g_soko_sub_idx   = 0;      // 当前正在执行的子解索引
static uint16 g_soko_wp_idx    = 0;      // 当前路点索引
static uint8  g_soko_exec_init = 0;      // 推箱执行阶段是否已初始化

// 第二阶段分类映射表 (box_id → target_id, 由 OpenART 识别后填入)
static uint8  g_box_to_target[SOKOBAN_MAX_BOXES] = {0};
static uint8  g_observe_count = 0;  // 已观察箱子数

// 第三阶段炸弹推送用变量
static Point_t         g_bomb_pos;              // 当前选中的炸弹坐标
static Point_t         g_bomb_wall_pos;         // 炸弹推送目标墙体
static SokoActionSeq_t g_bomb_action_seq;       // 推炸弹动作序列
static SokoWaypointPath_t g_bomb_waypoints;     // 推炸弹路点
static uint16          g_bomb_wp_idx = 0;       // 当前路点索引

/**
 * @brief 从底盘里程计同步当前网格坐标到 g_player_pos
 */
static void sync_player_pos(void)
{
    chassis_pose_t pose = chassis_ctrl_get_pose();
    int8 gx = (int8)(pose.x_m / CHASSIS_GRID_CELL_SIZE_M + 0.5f);
    int8 gy = (int8)(pose.y_m / CHASSIS_GRID_CELL_SIZE_M + 0.5f);

    // 钳位到合法网格范围
    if (gx < 0)                     gx = 0;
    if (gx > (int8)CHASSIS_GRID_MAX_X) gx = (int8)CHASSIS_GRID_MAX_X;
    if (gy < 0)                     gy = 0;
    if (gy > (int8)CHASSIS_GRID_MAX_Y) gy = (int8)CHASSIS_GRID_MAX_Y;

    g_player_pos.x = gx;
    g_player_pos.y = gy;
}

/**
 * @brief 推箱子执行通用逻辑：按路点列表逐点导航
 * @return 1=所有箱子全部推完, 0=还在执行中
 */
static uint8 exec_push_waypoints(void)
{
    if (!is_navigating) {
        if (g_soko_wp_idx < g_soko_waypoints.count) {
            HAL_CHASSIS_MOVE_TO(g_soko_waypoints.points[g_soko_wp_idx].x,
                                g_soko_waypoints.points[g_soko_wp_idx].y);
            is_navigating = 1;
        }
        return 0;
    }

    if (!HAL_CHASSIS_IS_ARRIVED()) return 0;

    /* 当前路点已到达 */
    is_navigating = 0;
    g_soko_wp_idx++;

    if (g_soko_wp_idx < g_soko_waypoints.count) {
        HAL_CHASSIS_MOVE_TO(g_soko_waypoints.points[g_soko_wp_idx].x,
                            g_soko_waypoints.points[g_soko_wp_idx].y);
        is_navigating = 1;
        return 0;
    }

    /* 当前子解路点走完 → 换下一个子解 */
    g_soko_sub_idx++;
    if (g_soko_sub_idx >= g_soko_solution.total_boxes) {
        return 1;   /* 全部推完 */
    }

    /* 生成下一个子解的路点 */
    Point_t start = g_soko_solution.player_end_pos[g_soko_sub_idx - 1];
    Sokoban_Actions_To_Waypoints(
        g_soko_solution.sub_solutions[g_soko_sub_idx].actions,
        g_soko_solution.sub_solutions[g_soko_sub_idx].count,
        start, &g_soko_waypoints);
    g_soko_wp_idx = 0;

    if (g_soko_waypoints.count > 0) {
        HAL_CHASSIS_MOVE_TO(g_soko_waypoints.points[0].x,
                            g_soko_waypoints.points[0].y);
        is_navigating = 1;
    }
    return 0;
}

/**
 * @brief 游戏主逻辑任务 (需放在 main 函数的 while(1) 中循环调用)
 */
void Game_Logic_Task_Run(void) {
    // 每轮循环先同步底盘里程计位姿到网格坐标
    sync_player_pos();
    
    switch (current_stage) {
        // -----------------------------------------------------------------
        // 【状态 1】：去最近的箱子看一眼，确定到底是第一阶段还是后面阶段
        // -----------------------------------------------------------------
        case STAGE_PENDING_SCOUT: {
            if (!is_navigating) {
                // 1. 找最近的箱子观察点
                Point_t target_box;
                current_target = Algo_Find_Nearest_Box_Observe_Point(g_game_map, g_player_pos, &target_box);
                
                if (current_target.is_valid) {
                    // 下发移动指令给底盘
                    HAL_CHASSIS_MOVE_TO(current_target.pos.x, current_target.pos.y);
                    is_navigating = 1;
                } else {
                    // 异常：地图上没有合法箱子，保持待机
                }
            } else {
                // 2. 检查是否已经开到观察点
                if (HAL_CHASSIS_IS_ARRIVED()) {
                    is_navigating = 0;
                    
                    // 3. 到了！抬头看前向摄像头(主镜头)的识别结果
                    uint8 class_id = HAL_VISION_GET_BOX_CLASS_ID();
                    
                    if (class_id == 0) {
                        // 没贴图，是基础模式，直接开推！
                        current_stage = STAGE_1_BASIC_EXEC;
                        g_soko_exec_init = 0;
                    } else {
                        // 有贴图！是第二或第三阶段。
                        // 规则要求：需要观察完所有的箱子建立映射。跳转到遍历状态！
                        current_stage = STAGE_OBSERVE_ALL;
                        // TODO: 将当前看过的这个箱子的 ID 记录到字典数组中
                    }
                }
            }
            break;
        }

        // -----------------------------------------------------------------
        // 【状态 2】：由于是二/三阶段，必须遍历全场尚未识别的箱子
        // -----------------------------------------------------------------
        case STAGE_OBSERVE_ALL: {
            if (!is_navigating) {
                /*
                 * 步骤 1: 扫描全局地图，寻找下一个“还没被看过的箱子”。
                 * 步骤 2: 调用 BFS 导航至其观察点。
                 * 步骤 3: 如果所有箱子都看完了，检查地图有无炸弹：
                 * 无炸弹 -> current_stage = STAGE_2_CLASS_EXEC;
                 * 有炸弹 -> current_stage = STAGE_3_STRATEGY_EXEC;
                 */
                 // (此处省略遍历字典的具体登记代码)
            } else {
                if (HAL_CHASSIS_IS_ARRIVED()) {
                    is_navigating = 0;
                    // 记录摄像头看到的 class_id，标记该箱子为已观察。
                }
            }
            break;
        }

        // -----------------------------------------------------------------
        // 【状态 3】：执行第一阶段（基础推箱）
        // -----------------------------------------------------------------
        case STAGE_1_BASIC_EXEC: {
            if (!g_soko_exec_init) {
                if (Sokoban_Solve_Stage1(g_game_map, g_player_pos,
                                          &g_soko_solution)) {
                    g_soko_sub_idx = 0;
                    Sokoban_Actions_To_Waypoints(
                        g_soko_solution.sub_solutions[0].actions,
                        g_soko_solution.sub_solutions[0].count,
                        g_player_pos, &g_soko_waypoints);
                    g_soko_wp_idx = 0;
                }
                g_soko_exec_init = 1;
            }
            if (g_soko_solution.is_solved) {
                exec_push_waypoints();
            }
            break;
        }

        // -----------------------------------------------------------------
        // 【状态 4】：执行第二阶段（分类推箱）
        // -----------------------------------------------------------------
        case STAGE_2_CLASS_EXEC: {
            if (!g_soko_exec_init) {
                if (Sokoban_Solve_Stage2(g_game_map, g_player_pos,
                                          g_box_to_target,
                                          g_soko_solution.total_boxes,
                                          &g_soko_solution)) {
                    g_soko_sub_idx = 0;
                    Sokoban_Actions_To_Waypoints(
                        g_soko_solution.sub_solutions[0].actions,
                        g_soko_solution.sub_solutions[0].count,
                        g_player_pos, &g_soko_waypoints);
                    g_soko_wp_idx = 0;
                }
                g_soko_exec_init = 1;
            }
            if (g_soko_solution.is_solved) {
                exec_push_waypoints();
            }
            break;
        }

        // -----------------------------------------------------------------
        // 【状态 5】：执行第三阶段（含炸弹策略推箱）
        //
        // 流程：
        //   1. 尝试 Stage2 求解（忽略炸弹，先推能推的）
        //   2. 若成功 → 按路点推箱（推完 → STAGE_DONE）
        //   3. 若无解 → 扫描地图找到不可达的目标
        //      a. 调用 Sokoban_Find_Bomb_Wall 选定最优爆破墙体
        //      b. 在地图中找到炸弹坐标
        //      c. 调用 Sokoban_Solve_Push_Bomb 求解推炸弹路径
        //      d. 转入 STAGE_3_BOMB_PUSH 子状态执行推炸弹
        // -----------------------------------------------------------------
        case STAGE_3_STRATEGY_EXEC: {
            if (!g_soko_exec_init) {
                uint8 ok = Sokoban_Solve_Stage2(g_game_map, g_player_pos,
                                                 g_box_to_target,
                                                 g_soko_solution.total_boxes,
                                                 &g_soko_solution);
                if (!ok) {
                    /* ---- 无解：需要炸弹清障 ---- */

                    /* 1. 扫描地图找到所有目标, 检测哪个不可达 */
                    Point_t blocked_target = {-1, -1};
                    {
                        NavPath_t nav_tmp;
                        for (int8 r = 0; r < MAP_ROWS; r++) {
                            for (int8 c = 0; c < MAP_COLS; c++) {
                                if (g_game_map[r][c] == MAP_TARGET) {
                                    Point_t tp = {c, r};
                                    if (!Algo_Nav_BFS(g_game_map, g_player_pos, tp, &nav_tmp)) {
                                        blocked_target = tp;
                                        break;
                                    }
                                }
                            }
                            if (blocked_target.x >= 0) break;
                        }
                    }

                    if (blocked_target.x >= 0) {
                        /* 2. 找最优爆破墙体 */
                        if (Sokoban_Find_Bomb_Wall(g_game_map, g_player_pos,
                                                    blocked_target, &g_bomb_wall_pos)) {
                            /* 3. 在地图中找炸弹 */
                            g_bomb_pos.x = -1;
                            for (int8 r = 0; r < MAP_ROWS && g_bomb_pos.x < 0; r++) {
                                for (int8 c = 0; c < MAP_COLS; c++) {
                                    if (g_game_map[r][c] == MAP_BOMB) {
                                        g_bomb_pos.x = c;
                                        g_bomb_pos.y = r;
                                        break;
                                    }
                                }
                            }

                            if (g_bomb_pos.x >= 0) {
                                /* 4. 求解推炸弹路径 */
                                if (Sokoban_Solve_Push_Bomb(g_game_map, g_player_pos,
                                                             g_bomb_pos, g_bomb_wall_pos,
                                                             &g_bomb_action_seq)) {
                                    Sokoban_Actions_To_Waypoints(
                                        g_bomb_action_seq.actions,
                                        g_bomb_action_seq.count,
                                        g_player_pos, &g_bomb_waypoints);
                                    g_bomb_wp_idx = 0;
                                    is_navigating = 0;
                                    current_stage = STAGE_3_BOMB_PUSH;
                                    break;  /* 跳出 switch, 下次进入 BOMB_PUSH */
                                }
                            }
                        }
                    }
                    /* 如果炸弹流程都失败了, 只能标记为无解, 继续等待 */
                }

                if (g_soko_solution.is_solved) {
                    g_soko_sub_idx = 0;
                    Sokoban_Actions_To_Waypoints(
                        g_soko_solution.sub_solutions[0].actions,
                        g_soko_solution.sub_solutions[0].count,
                        g_player_pos, &g_soko_waypoints);
                    g_soko_wp_idx = 0;
                }
                g_soko_exec_init = 1;
            }
            if (g_soko_solution.is_solved) {
                if (exec_push_waypoints()) {
                    current_stage = STAGE_DONE;
                }
            }
            break;
        }

        // -----------------------------------------------------------------
        // 【状态 6】：第三阶段子状态 — 推炸弹到目标墙体
        //
        // 按 g_bomb_waypoints 逐点导航, 推完后：
        //   1. 在地图上执行爆炸 (3×3 清除)
        //   2. 清除炸弹原始格
        //   3. 重置求解标志, 回到 STAGE_3_STRATEGY_EXEC 重试
        // -----------------------------------------------------------------
        case STAGE_3_BOMB_PUSH: {
            if (!is_navigating) {
                if (g_bomb_wp_idx < g_bomb_waypoints.count) {
                    HAL_CHASSIS_MOVE_TO(g_bomb_waypoints.points[g_bomb_wp_idx].x,
                                        g_bomb_waypoints.points[g_bomb_wp_idx].y);
                    is_navigating = 1;
                }
            } else {
                if (HAL_CHASSIS_IS_ARRIVED()) {
                    is_navigating = 0;
                    g_bomb_wp_idx++;

                    if (g_bomb_wp_idx < g_bomb_waypoints.count) {
                        HAL_CHASSIS_MOVE_TO(g_bomb_waypoints.points[g_bomb_wp_idx].x,
                                            g_bomb_waypoints.points[g_bomb_wp_idx].y);
                        is_navigating = 1;
                    } else {
                        /* 推炸弹完成 → 执行爆炸 */
                        Sokoban_Apply_Bomb_Explosion(g_game_map, g_bomb_wall_pos);
                        g_game_map[g_bomb_pos.y][g_bomb_pos.x] = MAP_EMPTY;  // 清除炸弹原格

                        /* 重置求解标志, 回到 STAGE_3 重新尝试分类推箱 */
                        g_soko_exec_init = 0;
                        is_navigating = 0;
                        current_stage = STAGE_3_STRATEGY_EXEC;
                    }
                }
            }
            break;
        }

        // -----------------------------------------------------------------
        // 【状态 7】：全部完成
        // -----------------------------------------------------------------
        case STAGE_DONE: {
            /* 任务完成, 停车等待 */
            break;
        }

        default:
            break;
    }
}