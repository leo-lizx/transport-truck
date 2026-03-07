#include "app_game_logic.h"

/**
 * @brief  根据视觉统计信息识别当前游戏阶段
 * @param  stats: 视觉串口解析模块传递过来的地图宏观统计信息
 * @return GameStage_e: 返回当前判定的游戏阶段
 */
GameStage_e Game_Identify_Stage(VisionStats_t *stats) {
    if (stats == NULL) return STAGE_UNKNOWN;

    // 逻辑1：如果地图中存在炸弹，必定是第三阶段（策略模式）
    if (stats->bomb_count > 0) {
        return STAGE_3_STRATEGY;
    }
    // 逻辑2：如果没有炸弹，但视觉系统提示需要进行图像与数字分类匹配，则是第二阶段
    else if (stats->need_classification > 0) {
        return STAGE_2_CLASS;
    }
    // 逻辑3：既没有炸弹也不需要分类，且场上有箱子，则是第一阶段基础模式
    else if (stats->box_count > 0) {
        return STAGE_1_BASIC;
    }

    return STAGE_UNKNOWN;
}

/**
 * @brief  执行对应阶段的顶层策略调度 (状态机分支)
 * @param  stage: 当前识别出的游戏阶段
 */
void Game_Execute_Strategy(GameStage_e stage) {
    switch (stage) {
        case STAGE_1_BASIC:
            /* * 策略一流程：
             * 1. 将地图降维分解为单箱地图
             * 2. 调用 BFS 算法解算 推箱路径
             * 3. (暂不编写) 驱动底盘执行路径
             */
            // Algo_Sokoban_Solve(...); // 留作后续调用推箱子算法
            break;

        case STAGE_2_CLASS:
            /* * 策略二流程：
             * 1. 寻找箱子与目标点的侧面观察点
             * 2. (暂不编写) 驱动车模前往观察点，等待 OpenART#2 识别结果
             * 3. 建立箱子与目标的正确映射关系
             * 4. 再次调用第一阶段的推箱逻辑
             */
            break;

        case STAGE_3_STRATEGY:
            /* * 策略三流程：
             * 1. 遍历地图内部墙体，进行炸点评估解算
             * 2. 调用 BFS 解算推炸弹的最优路径
             * 3. (暂不编写) 驱动底盘引爆炸弹更新地图
             * 4. 再次调用第二阶段/第一阶段推箱逻辑
             */
            break;

        case STAGE_UNKNOWN:
        default:
            // 地图尚未加载或异常，保持原地等待
            break;
    }
}