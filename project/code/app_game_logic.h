#ifndef _APP_GAME_LOGIC_H_
#define _APP_GAME_LOGIC_H_

#include "zf_common_headfile.h" // 包含逐飞通用头文件

// 游戏阶段枚举
typedef enum {
    STAGE_UNKNOWN = 0,   // 未知阶段（地图未识别）
    STAGE_1_BASIC,       // 阶段一：基础推箱子
    STAGE_2_CLASS,       // 阶段二：分类推箱子
    STAGE_3_STRATEGY     // 阶段三：策略推箱子（含炸弹）
} GameStage_e;

// 视觉地图统计信息结构体（这部分数据将由串口接收模块解析后填充）
typedef struct {
    uint8 box_count;           // 场上存在的箱子总数
    uint8 bomb_count;          // 场上存在的炸弹总数
    uint8 need_classification; // 是否需要分类识别 (1:需要匹配图像, 0:不需要)
} VisionStats_t;

// 函数声明
GameStage_e Game_Identify_Stage(VisionStats_t *stats);
void Game_Execute_Strategy(GameStage_e stage);

#endif