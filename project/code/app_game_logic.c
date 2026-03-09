#include "app_game_logic.h"

uint8 g_game_map[MAP_ROWS][MAP_COLS];
Point_t g_player_pos = {0, 0}; // 车模实时坐标

// 状态机内部使用的静态变量
static GameStage_e current_stage = STAGE_PENDING_SCOUT;
static uint8 is_navigating = 0;       // 是否正在移动的标志位
static ObservePoint_t current_target; // 当前需要前往的观察点

/**
 * @brief 游戏主逻辑任务 (需放在 main 函数的 while(1) 中循环调用)
 */
void Game_Logic_Task_Run(void) {
    
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
            // 在此调用您之前的推箱子 BFS 解算算法
            // 按照路径输出的 ACT_UP, ACT_DOWN 逐格控制底盘
            break;
        }

        // -----------------------------------------------------------------
        // 【状态 4 & 5】：执行第二/第三阶段（分类/策略推箱）
        // -----------------------------------------------------------------
        case STAGE_2_CLASS_EXEC:
        case STAGE_3_STRATEGY_EXEC: {
            // 利用刚才在 STAGE_OBSERVE_ALL 中建立好的“图片-数字”字典
            // 严格匹配目标后，调用推箱子/炸毁墙体逻辑
            break;
        }

        default:
            break;
    }
}