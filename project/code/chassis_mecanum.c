/*===========================================================================
 * [chassis_mecanum.c] 麦克纳姆轮运动学实现
 *===========================================================================*/

#include "chassis_mecanum.h"
#include <math.h>

/**
 * @brief  正运动学：车体速度映射到四轮线速度
 */
void chassis_mecanum_forward(float vx_body_mps,
                             float vy_body_mps,
                             float wz_radps,
                             float out_wheel_mps[CHASSIS_WHEEL_COUNT])
{
    /*
     * 麦轮正运动学公式 (X / O 型由 CHASSIS_MECANUM_O_TYPE_LAYOUT 控制):
     *
     *   X 型 (辊子在车顶画 X):
     *     v_LF = vy + vx - K·wz
     *     v_RF = vy - vx + K·wz
     *     v_LB = vy - vx - K·wz
     *     v_RB = vy + vx + K·wz
     *
     *   O 型 (辊子在车顶画 O / 菱形, 当前实车): vx 项整体翻号
     *     v_LF = vy - vx - K·wz
     *     v_RF = vy + vx + K·wz
     *     v_LB = vy + vx - K·wz
     *     v_RB = vy - vx + K·wz
     *
     * 物理含义 (两种布局共通):
     *   vy (前进) 对所有轮子产生等同正向速度
     *   vx (横移) 使左右两侧轮产生相反速度, 符号差异源于辊子方向
     *   wz (旋转) 使对角线轮产生相反速度, K = 半轴距 + 半轮距
     */
#if (CHASSIS_MECANUM_O_TYPE_LAYOUT != 0)
    const float vx_sign = -1.0f;   /* O 型: vx 整体翻号 */
#else
    const float vx_sign = +1.0f;   /* X 型: 标准方向 */
#endif

    out_wheel_mps[CHASSIS_WHEEL_LF] = vy_body_mps + vx_sign * vx_body_mps - CHASSIS_MECANUM_K_M * wz_radps;
    out_wheel_mps[CHASSIS_WHEEL_RF] = vy_body_mps - vx_sign * vx_body_mps + CHASSIS_MECANUM_K_M * wz_radps;
    out_wheel_mps[CHASSIS_WHEEL_LB] = vy_body_mps - vx_sign * vx_body_mps - CHASSIS_MECANUM_K_M * wz_radps;
    out_wheel_mps[CHASSIS_WHEEL_RB] = vy_body_mps + vx_sign * vx_body_mps + CHASSIS_MECANUM_K_M * wz_radps;
}

/**
 * @brief  逆运动学：四轮线速度反解车体平移速度
 */
void chassis_mecanum_inverse(const float wheel_mps[CHASSIS_WHEEL_COUNT],
                             float *out_vx_mps,
                             float *out_vy_mps)
{
    /*
     * 逆运动学: 四轮速度取平均反解车体速度
     *
     *   X 型: vx_body = (v_LF - v_RF - v_LB + v_RB) / 4
     *   O 型: vx_body = (-v_LF + v_RF + v_LB - v_RB) / 4   (整体翻号)
     *
     *   vy_body 公式两种布局相同:
     *           vy_body = (v_LF + v_RF + v_LB + v_RB) / 4
     */
#if (CHASSIS_MECANUM_O_TYPE_LAYOUT != 0)
    *out_vx_mps = (-wheel_mps[CHASSIS_WHEEL_LF] + wheel_mps[CHASSIS_WHEEL_RF]
                 + wheel_mps[CHASSIS_WHEEL_LB] - wheel_mps[CHASSIS_WHEEL_RB]) * 0.25f;
#else
    *out_vx_mps = (wheel_mps[CHASSIS_WHEEL_LF] - wheel_mps[CHASSIS_WHEEL_RF]
                 - wheel_mps[CHASSIS_WHEEL_LB] + wheel_mps[CHASSIS_WHEEL_RB]) * 0.25f;
#endif

    *out_vy_mps = (wheel_mps[CHASSIS_WHEEL_LF] + wheel_mps[CHASSIS_WHEEL_RF]
                 + wheel_mps[CHASSIS_WHEEL_LB] + wheel_mps[CHASSIS_WHEEL_RB]) * 0.25f;
}

/**
 * @brief  四轮等比例限速（保持速度方向不变）
 */
void chassis_mecanum_clamp_wheels(float wheel_mps[CHASSIS_WHEEL_COUNT],
                                  float max_speed)
{
    float max_abs_wheel_speed = 0.0f; /* 四轮中绝对值最大的速度 */
    float speed_scale;                /* 超限时统一缩放系数 */
    uint8 i;                          /* 轮序号 */

    /* 找出四轮中绝对值最大的速度 */
    for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
    {
        float abs_val = fabsf(wheel_mps[i]);
        if (abs_val > max_abs_wheel_speed)
        {
            max_abs_wheel_speed = abs_val;
        }
    }

    /* 若超限：所有轮乘以同一缩放系数，保持运动方向不变 */
    if (max_abs_wheel_speed > max_speed && max_abs_wheel_speed > 1e-6f)
    {
        speed_scale = max_speed / max_abs_wheel_speed;
        for (i = 0U; i < (uint8)CHASSIS_WHEEL_COUNT; ++i)
        {
            wheel_mps[i] *= speed_scale;
        }
    }
}
