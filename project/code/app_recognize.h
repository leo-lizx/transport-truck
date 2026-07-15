/*********************************************************************************************************************
 * 文件名称   : app_recognize.h
 * 模块功能   : 推箱子识别 tour 子状态机 (Stage2 / Stage3)
 *--------------------------------------------------------------------------------------------------------------------
 * 用途:
 *   游戏主状态机 STAGE_RECOGNIZE_MAP 的真正内容由本模块承担:
 *     1) 提取地图中所有 BOX 与 TARGET 坐标
 *     2) 小规模精确 tour / BFS 最近观察点计算访问顺序 (与 PC 验证器 plan_scout_phase_v2 对齐)
 *     3) 对每个待识别物体:
 *          - 移动到侧面观察点 (BFS 选最近的可立足格)
 *          - 最终观察航点用视觉 Snap 校准到格中心
 *          - 原地旋转车头朝向物体
 *          - 等待视觉端 BOX_CLASS 帧持续广播, 多数票稳定后采样
 *     4) 全部物体识别完成后, 把 (boxIdx -> targetIdx) 映射写入 g_box_to_target[]
 *        游戏主状态机据此进入 STAGE_PLAN_PATH (Sokoban_Solve_Stage2)
 *
 * 输出:
 *   - g_box_to_target[i] : 第 i 个箱子 (按 extract 顺序) 应推到第 ? 号目标 (按 extract 顺序)
 *   - App_Recognize_Tick 返回值见 AppRecognizeStatus_e, 业务侧据此切 stage
 *
 * 与硬件层契约:
 *   - 移动:    chassis_ctrl_move_to_m(基准 yaw) / chassis_ctrl_is_arrived / 到站视觉 Snap
 *   - 朝向:    chassis_ctrl_rotate_to_deg  /  chassis_ctrl_is_arrived
 *   - 视觉:    app_link_get_box_class_snapshot (seq-lock 拷贝)
 *
 * 调用者:
 *   - app_game_logic.c::stage_recognize_handler() 每 5ms 一次
 *   - 链路掉线时由上层把状态机切到 STAGE_PAUSE_ON_LINK_LOSS, 本模块在 INIT 状态下被
 *     冷启动, 链路恢复后自动重新走识别流程 (RECOGNIZE_MAP 是恢复点)
 *
 * 资源占用:
 *   - 全部 static, BSS 约 6KB; 单循环线程, 不可重入, 不可在 ISR 调用
 *********************************************************************************************************************/
#ifndef _APP_RECOGNIZE_H_
#define _APP_RECOGNIZE_H_

#include "zf_common_typedef.h"
#include "algo_sokoban_solver.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===================================================================================================================
 * 状态返回值 — 由 App_Recognize_Tick() 返回, 上层据此决定下一 stage
 *=================================================================================================================*/
typedef enum
{
    APP_RECOG_RUNNING = 0,    /* 仍在识别中, 继续 tick                                  */
    APP_RECOG_DONE_OK,        /* 全部物体识别完成, 映射已写入 g_box_to_target[]         */
    APP_RECOG_DONE_NO_NEED,   /* 当前关卡不需要识别 (如 Stage1 简单贪心), 直接进 PLAN  */
    APP_RECOG_FAIL            /* 物体不可达 / 视觉持续无识别 → 上层应切 DEADLOCK_RESET */
} AppRecognizeStatus_e;

/*===================================================================================================================
 * 子状态枚举 — 仅供调试观测 (菜单/IPS), 不要求外部业务依赖具体值
 *=================================================================================================================*/
typedef enum
{
    RECOG_SUB_INIT     = 0,   /* 初始化 / 提取物体列表 / 决定是否需要识别 */
    RECOG_SUB_NAV,            /* 移动到当前物体观察点                     */
    RECOG_SUB_FACE,            /* 原地旋转车头朝向物体                     */
    RECOG_SUB_SAMPLE,          /* 多数票采样视觉分类结果                   */
    RECOG_SUB_RETURN_YAW,      /* 旧调试枚举: 采样后保持当前 yaw, 不再回正 */
    RECOG_SUB_NEXT,            /* 当前物体完成, 切下一个                   */
    RECOG_SUB_DONE,            /* 全部完成 (对外输出 DONE_OK 一帧后归 INIT)*/
    RECOG_SUB_FAIL             /* 不可达或视觉超时                         */
} AppRecognizeSub_e;

/*===================================================================================================================
 * 调试观察结构体 — 给菜单/上位机用
 *=================================================================================================================*/
typedef struct
{
    AppRecognizeSub_e sub_state;
    uint8  total_targets;       /* 本轮待识别物体总数 (boxes + targets) */
    uint8  current_idx;         /* 当前正在识别第几个                    */
    uint8  current_kind;        /* APP_LINK_OBJ_KIND_BOX / TARGET        */
    uint8  current_class_id;    /* 当前多数票得到的 class_id (0=未确定)  */
    uint16 sample_count;        /* 已采样次数 (调试用)                   */
    uint8  visited_count;       /* 已实测访问识别完成数 (含跳过失败)     */
    uint8  inferred_count;      /* 保留字段, 恒为 0                        */
    uint8  resolved_box;        /* 已确定 class_id 的箱子数              */
    uint8  resolved_target;     /* 已确定 class_id 的目标点数            */
} AppRecognizeDebug_t;

/*===================================================================================================================
 * 对外 API
 *=================================================================================================================*/

/**
 * @brief 复位识别状态机, 强制下次 Tick 从 INIT 重新跑
 *        - 进入 RECOGNIZE_MAP 时由上层调用一次
 *        - 链路掉线恢复后由上层在 RECOGNIZE_MAP 入口调用一次
 */
void App_Recognize_Reset(void);

/**
 * @brief 5ms 周期 tick — 推进识别子状态机
 *
 * @param map           当前地图快照 (主控侧 g_game_map); 清障推箱成功后会原地更新
 * @param player_pos    当前车体网格坐标
 * @param has_bomb      地图是否含炸弹 (= map_has_bomb 结果)
 * @param level         当前关卡 (1=Stage1 跳过识别, ≥2 = 必走识别)
 * @param box_to_target_out  [out] 长度 SOKOBAN_MAX_BOXES, DONE_OK 时被填充
 *
 * @return AppRecognizeStatus_e
 */
AppRecognizeStatus_e App_Recognize_Tick(uint8 map[MAP_ROWS][MAP_COLS],
                                        Point_t player_pos,
                                        uint8 has_bomb,
                                        uint8 level,
                                        uint8 box_to_target_out[SOKOBAN_MAX_BOXES]);

/**
 * @brief 取调试信息 (供菜单/IPS显示)
 */
void App_Recognize_Get_Debug(AppRecognizeDebug_t *out);

/**
 * @brief 本轮识别是否为打通观察路线而移动过箱子。
 *        返回 1 时, 上层应至少保持本地图到后续推箱计划生成完毕。
 */
uint8 App_Recognize_Map_Changed(void);

#ifdef __cplusplus
}
#endif

#endif /* _APP_RECOGNIZE_H_ */
