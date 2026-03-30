#ifndef CHASSIS_ENCODER_H
#define CHASSIS_ENCODER_H

/*===========================================================================
 * [chassis_encoder.h] 编码器测速模块
 *
 *   封装正交编码器（A/B 双通道）的初始化和测速逻辑。
 *   每个编码器有两条信号线：
 *     A 相（CH1）— 脉冲计数
 *     B 相（CH2）— 方向判断
 *   MCU 定时器硬件自动检测旋转方向和脉冲数。
 *
 *   速度计算公式：
 *     轮速(m/s) = 脉冲增量 × (2πR / 每转脉冲数) / 采样周期
 *   其中 R = CHASSIS_WHEEL_RADIUS_M（见 chassis_config.h）
 *
 *   使用流程：
 *     1. chassis_encoder_init()   — 初始化编码器计数器
 *     2. chassis_encoder_update() — 每 20ms 调用，读取增量并换算速度
 *     3. chassis_encoder_get_speed() — 获取最近一次速度读数
 *===========================================================================*/

#include "chassis_config.h"

/** 单个编码器的配置与状态 */
typedef struct
{
    encoder_index_enum    index;     /**< 编码器硬件通道编号 */
    encoder_channel1_enum ch1_pin;   /**< A 相（CH1）信号引脚 */
    encoder_channel2_enum ch2_pin;   /**< B 相（CH2）信号引脚 */
    float                 dir_sign;  /**< 编码器方向修正系数（由 CHASSIS_*_ENC_SIGN 配置） */
    float                 speed_mps; /**< 最近一次读取的轮速（m/s），正值=正向 */
} chassis_encoder_t;

/**
 * @brief  初始化编码器硬件（配置计数器和引脚）
 * @param  enc  编码器结构体指针
 */
void chassis_encoder_init(chassis_encoder_t *enc);

/**
 * @brief  读取编码器脉冲增量并更新轮速
 *         应在固定周期任务中调用（如 20ms 周期）
 * @param  enc   编码器结构体指针
 * @param  dt_s  采样周期（秒），通常传 CHASSIS_TASK_DT_20MS_S
 */
void chassis_encoder_update(chassis_encoder_t *enc, float dt_s);

/**
 * @brief  获取最近一次读取的轮速
 * @param  enc  编码器结构体指针
 * @return      轮速（m/s），正值表示正向旋转
 */
float chassis_encoder_get_speed(const chassis_encoder_t *enc);

#endif /* CHASSIS_ENCODER_H */
