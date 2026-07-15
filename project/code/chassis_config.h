#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/*===========================================================================
 * [chassis_config.h] 底盘配置聚合头
 *
 *   原大而全的 1129 行单文件已按 HecateFlow 三层分离法拆分为:
 *     config/pinMap.h        — 引脚层 (零依赖纯 #define)
 *     config/configChassis.h — 参数层 (分节 §A~§F，含 §极性段)
 *
 *   本文件保留: 驱动头依赖 + 枚举类型 + static inline 工具函数。
 *   所有模块仍 #include "chassis_config.h" 即可，无需逐个改依赖。
 *
 *   [车轮编号约定] (俯视图，车头朝上):
 *          ┌── 车头(前方)──┐
 *          │  LF        RF  │    LF = Left-Front  左前轮
 *          │  LB        RB  │    RF = Right-Front 右前轮
 *          └────────────────┘    LB = Left-Back   左后轮
 *                                RB = Right-Back  右后轮
 *
 *   [坐标系约定]:
 *     车体坐标系: X 向右为正，Y 向前为正
 *     全局坐标系: X 向右为正，Y 向前为正
 *     航向角 yaw: 逆时针为正 (从 X 轴到 Y 轴方向)
 *===========================================================================*/

#include "zf_common_headfile.h"
#include "config/pinMap.h"
#include "config/configChassis.h"

/* ======================================================================
 *  轮子编号枚举 — 统一四轮索引，用于数组下标
 * ====================================================================== */

typedef enum
{
    CHASSIS_WHEEL_LF = 0,   /**< 左前轮 */
    CHASSIS_WHEEL_RF,       /**< 右前轮 */
    CHASSIS_WHEEL_LB,       /**< 左后轮 */
    CHASSIS_WHEEL_RB,       /**< 右后轮 */
    CHASSIS_WHEEL_COUNT     /**< 轮子总数 = 4，用于数组大小 */
} chassis_wheel_index_t;

/* ======================================================================
 *  通用工具函数 (static inline，头文件内联，各模块可直接调用)
 * ====================================================================== */

/**
 * @brief  浮点数限幅(钳位)
 * @param  value    输入值
 * @param  min_val  下限
 * @param  max_val  上限
 * @return          限幅后的值，保证在 [min_val, max_val] 范围内
 */
static inline float chassis_clamp_f(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief  将 X 网格索引钳位到可通行内场范围 [1, 14]
 */
static inline uint8 chassis_clamp_grid_x_inner(uint8 grid_x)
{
    if (grid_x < CHASSIS_GRID_INNER_MIN_X) return CHASSIS_GRID_INNER_MIN_X;
    if (grid_x > CHASSIS_GRID_INNER_MAX_X) return CHASSIS_GRID_INNER_MAX_X;
    return grid_x;
}

/**
 * @brief  将 Y 网格索引钳位到可通行内场范围 [1, 10]
 */
static inline uint8 chassis_clamp_grid_y_inner(uint8 grid_y)
{
    if (grid_y < CHASSIS_GRID_INNER_MIN_Y) return CHASSIS_GRID_INNER_MIN_Y;
    if (grid_y > CHASSIS_GRID_INNER_MAX_Y) return CHASSIS_GRID_INNER_MAX_Y;
    return grid_y;
}

/**
 * @brief  可通行网格 X 索引 -> 车体中心物理坐标 X (米)
 *         以可通行区域左边界为 0m，整数格返回对应格中心。
 */
static inline float chassis_grid_x_to_m(uint8 grid_x)
{
    int32 inner_x = (int32)chassis_clamp_grid_x_inner(grid_x) - (int32)CHASSIS_GRID_INNER_MIN_X;
    return (((float)inner_x + 0.5f) * CHASSIS_GRID_STEP_X_M);
}

/**
 * @brief  可通行网格 Y 索引 -> 车体中心物理坐标 Y (米)
 *         以可通行区域上边界为 0m，整数格返回对应格中心。
 */
static inline float chassis_grid_y_to_m(uint8 grid_y)
{
    int32 inner_y = (int32)chassis_clamp_grid_y_inner(grid_y) - (int32)CHASSIS_GRID_INNER_MIN_Y;
    return (((float)inner_y + 0.5f) * CHASSIS_GRID_STEP_Y_M);
}

/**
 * @brief  车体中心物理坐标 X (米) -> 可通行网格 X 索引
 */
static inline uint8 chassis_m_to_grid_x(float x_m)
{
    int32 grid_x = (int32)(x_m / CHASSIS_GRID_STEP_X_M) + (int32)CHASSIS_GRID_INNER_MIN_X;
    if (grid_x < (int32)CHASSIS_GRID_INNER_MIN_X) grid_x = (int32)CHASSIS_GRID_INNER_MIN_X;
    if (grid_x > (int32)CHASSIS_GRID_INNER_MAX_X) grid_x = (int32)CHASSIS_GRID_INNER_MAX_X;
    return (uint8)grid_x;
}

/**
 * @brief  车体中心物理坐标 Y (米) -> 可通行网格 Y 索引
 */
static inline uint8 chassis_m_to_grid_y(float y_m)
{
    int32 grid_y = (int32)(y_m / CHASSIS_GRID_STEP_Y_M) + (int32)CHASSIS_GRID_INNER_MIN_Y;
    if (grid_y < (int32)CHASSIS_GRID_INNER_MIN_Y) grid_y = (int32)CHASSIS_GRID_INNER_MIN_Y;
    if (grid_y > (int32)CHASSIS_GRID_INNER_MAX_Y) grid_y = (int32)CHASSIS_GRID_INNER_MAX_Y;
    return (uint8)grid_y;
}

/**
 * @brief  角度归一化到 [-180, +180) 范围
 *         避免角度跳变 (如从 +179° 突变到 -179°) 导致控制突变
 * @param  angle_deg  输入角度 (度)
 * @return            归一化后的角度 (度)
 */
static inline float chassis_normalize_angle_deg(float angle_deg)
{
    while (angle_deg >  180.0f) { angle_deg -= 360.0f; }
    while (angle_deg < -180.0f) { angle_deg += 360.0f; }
    return angle_deg;
}

/**
 * @brief  航向吸附到最近的 90° 倍数 (0 / ±90 / 180)
 *         网格导航按轴对齐控制器设计: 保持任意斜角 (如识别转向后的 37°)
 *         平移会让两轴速度耦合, 精度和限速表现都变差。派发航点前先 snap。
 * @param  yaw_deg  当前航向 (度, 任意范围)
 * @return          最近的基准航向 (度, [-90, 180] 内的 90° 倍数)
 */
static inline float chassis_snap_yaw_to_cardinal_deg(float yaw_deg)
{
    float norm = chassis_normalize_angle_deg(yaw_deg);
    float snapped = 90.0f * (float)(int32)((norm + ((norm >= 0.0f) ? 45.0f : -45.0f)) / 90.0f);
    return chassis_normalize_angle_deg(snapped);
}

#endif /* CHASSIS_CONFIG_H */
