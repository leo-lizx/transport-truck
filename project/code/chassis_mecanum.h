#ifndef CHASSIS_MECANUM_H
#define CHASSIS_MECANUM_H

/*===========================================================================
 * [chassis_mecanum.h] 麦克纳姆轮运动学模块
 *
 *   麦轮（Mecanum Wheel）可以实现全向移动（前后、左右、旋转）。
 *   本模块提供两个方向的运动学计算：
 *
 *   [正运动学] 车体速度 → 四轮速度（用于下发控制指令）
 *     输入：vx(右平移), vy(前进), wz(旋转)
 *     输出：4 个轮子的目标线速度
 *
 *   [逆运动学] 四轮速度 → 车体速度（用于里程计估算）
 *     输入：4 个轮子的实际线速度
 *     输出：vx_body, vy_body
 *
 *   正运动学公式（标准 O 型布局）：
 *     v_LF = vy - vx - K × wz
 *     v_RF = vy + vx + K × wz
 *     v_LB = vy + vx - K × wz
 *     v_RB = vy - vx + K × wz
 *   其中 K = CHASSIS_MECANUM_K_M = 半轴距 + 半轮距
 *
 *   逆运动学公式：
 *     vx_body = (-v_LF + v_RF + v_LB - v_RB) / 4
 *     vy_body = (v_LF + v_RF + v_LB + v_RB) / 4
 *===========================================================================*/

#include "chassis_config.h"

/**
 * @brief  正运动学：车体速度 → 四轮目标线速度
 * @param  vx_body_mps    车体 X 速度（m/s，向右为正）
 * @param  vy_body_mps    车体 Y 速度（m/s，向前为正）
 * @param  wz_radps       车体角速度（rad/s，逆时针为正）
 * @param  out_wheel_mps  输出数组 [LF, RF, LB, RB]，各轮线速度（m/s）
 */
void chassis_mecanum_forward(float vx_body_mps,
                             float vy_body_mps,
                             float wz_radps,
                             float out_wheel_mps[CHASSIS_WHEEL_COUNT]);

/**
 * @brief  逆运动学：四轮实际线速度 → 车体速度
 * @param  wheel_mps    输入数组 [LF, RF, LB, RB]，各轮实际线速度（m/s）
 * @param  out_vx_mps   输出：车体 X 速度（m/s）
 * @param  out_vy_mps   输出：车体 Y 速度（m/s）
 */
void chassis_mecanum_inverse(const float wheel_mps[CHASSIS_WHEEL_COUNT],
                             float *out_vx_mps,
                             float *out_vy_mps);

/**
 * @brief  四轮等比例限速
 *         若任一轮速超过 max_speed，所有轮按同一比例缩放，
 *         保证运动方向不变，只降低整体速度
 * @param  wheel_mps  输入/输出数组 [LF, RF, LB, RB]
 * @param  max_speed  单轮最大允许线速度（m/s）
 */
void chassis_mecanum_clamp_wheels(float wheel_mps[CHASSIS_WHEEL_COUNT],
                                  float max_speed);

#endif /* CHASSIS_MECANUM_H */
