/*===========================================================================
 * [chassis_imu.c] IMU 航向采样与积分模块实现
 *===========================================================================*/

#include "chassis_imu.h"
#include "chassis_config.h"
#include "zf_device_imu660rb.h"
#include "zf_driver_delay.h"
#include <math.h>

/* IMU 积分与滤波参数（统一使用配置头中的宏） */
#define IMU_DT_S                 CHASSIS_TASK_DT_5MS_S
#define IMU_GYRO_DEADZONE_DPS    CHASSIS_IMU_GYRO_DEADZONE_DPS
#define IMU_GYRO_LPF_ALPHA       CHASSIS_IMU_GYRO_LPF_ALPHA
#define IMU_BIAS_ADAPT_ALPHA     CHASSIS_IMU_BIAS_ADAPT_ALPHA
#define IMU_YAW_SIGN             CHASSIS_IMU_YAW_SIGN

/*
 * Yaw 角速度来源轴宏 (P0-修复 2026-04-29 IMU 装反):
 *   IMU 装反后水平面旋转不再投影到 Z 轴, 改为投影到 Y 轴 (或 X 轴);
 *   下面用 imu_yaw_gyro_raw_dps() 包裹, 由 chassis_config.h 的
 *   CHASSIS_IMU_YAW_AXIS 在编译期选择. 选项见 config 注释。
 *   原来直接用 imu660rb_gyro_z 的代码以注释形式保留, 方便回退。
 */
static inline float imu_yaw_gyro_raw_dps(void)
{
#if   (CHASSIS_IMU_YAW_AXIS == CHASSIS_IMU_YAW_AXIS_Z)
    return imu660rb_gyro_transition(imu660rb_gyro_z);   /* IMU 正装 */
#elif (CHASSIS_IMU_YAW_AXIS == CHASSIS_IMU_YAW_AXIS_Y)
    return imu660rb_gyro_transition(imu660rb_gyro_y);   /* IMU 侧装: 原 pitch 轴 */
#elif (CHASSIS_IMU_YAW_AXIS == CHASSIS_IMU_YAW_AXIS_X)
    return imu660rb_gyro_transition(imu660rb_gyro_x);   /* IMU 侧装: 原 roll  轴 */
#else
#error "CHASSIS_IMU_YAW_AXIS must be Z / Y / X"
#endif
}

/** 全局欧拉角输出（当前控制链只使用 yaw） */
volatile EulerAngle_t car_angle = {0.0f, 0.0f, 0.0f};

/** Z 轴陀螺仪静态零偏（单位：度/秒） */
static float s_gyro_z_bias_dps = 0.0f;

/** Z 轴角速度一阶低通状态（单位：度/秒） - 经死区, 用于 yaw 积分 */
static float s_yaw_rate_lpf_dps = 0.0f;

/* P0-修复 2026-04-29 段段 bug:
 * 给 D 项专用的“无死区”角速度 LPF。原 s_yaw_rate_lpf_dps 经死区(0.8 dps)处理,
 * 与 KD=300 组合时, 死区边界附近 D 项在 0 ↔ -240 之间反复跳, 直接造成
 * yaw_pi 输出“一段一段”。D 项需要的是连续可微的角速度估计, 不能再死区。
 * 此通道仅去 bias + LPF, 不做死区, 也不参与 yaw 积分。 */
static float s_yaw_rate_for_d_dps = 0.0f;

/* ----------------------------------------------------------------------
 *  滑窗静止检测 (P0-改进 2026-04-29 通用零偏在线辨识)
 *  原始 gyro 入循环 buffer, 增量维护和与平方和, 单拍 O(1) 算窗口均值/方差。
 *  当窗口方差 < CHASSIS_IMU_STILL_VAR_TH_DPS2 -> 视为真静止, 用窗口均值
 *  以 BIAS_FAST_ALPHA EMA 快速校正 bias; 否则不更新 bias。
 * ---------------------------------------------------------------------- */
static float  s_still_buf[CHASSIS_IMU_STILL_WINDOW_LEN] = {0};
static uint16 s_still_idx = 0U;
static uint16 s_still_count = 0U;       /* 已填入的样本数, 上限 = WINDOW_LEN */
static float  s_still_sum = 0.0f;        /* 窗口内样本和 */
static float  s_still_sum_sq = 0.0f;     /* 窗口内样本平方和 */
static uint8  s_imu_is_still = 0U;       /* 最近一次判别结果, 调试可读 */

/**
 * @brief  把一拍原始 gyro 喂进静止检测器, 回写当前窗口均值 / 方差
 * @param  raw_dps  本拍偏航轴原始角速度 (度/秒, 未补零偏)
 * @param  out_mean 输出: 当前窗口均值 (度/秒)
 * @param  out_var  输出: 当前窗口方差 (度/秒)^2
 * @return 1 = 窗口已填满且为静止; 0 = 窗口未满或仍在运动
 */
static uint8 still_detect_step(float raw_dps, float *out_mean, float *out_var)
{
    float old_sample;
    float mean;
    float var;

    /* 1) 滑窗增量维护: 用旧值减、新值加, 避免每拍 O(N) 求和 */
    if (s_still_count < CHASSIS_IMU_STILL_WINDOW_LEN)
    {
        /* 窗口未满, 直接累加 */
        s_still_buf[s_still_idx] = raw_dps;
        s_still_sum    += raw_dps;
        s_still_sum_sq += raw_dps * raw_dps;
        s_still_count++;
    }
    else
    {
        /* 窗口已满, 替换最旧样本 */
        old_sample = s_still_buf[s_still_idx];
        s_still_buf[s_still_idx] = raw_dps;
        s_still_sum    += raw_dps - old_sample;
        s_still_sum_sq += (raw_dps * raw_dps) - (old_sample * old_sample);
    }
    s_still_idx = (uint16)((s_still_idx + 1U) % CHASSIS_IMU_STILL_WINDOW_LEN);

    /* 2) 计算窗口均值和方差 (Var = E[x^2] - (E[x])^2) */
    mean = s_still_sum / (float)s_still_count;
    var  = (s_still_sum_sq / (float)s_still_count) - (mean * mean);
    if (var < 0.0f) { var = 0.0f; }   /* 数值噪声防负 */

    *out_mean = mean;
    *out_var  = var;

    /* 3) 仅在窗口填满后才出静止结论, 避免开机几十毫秒内误判 */
    if (s_still_count < CHASSIS_IMU_STILL_WINDOW_LEN) { return 0U; }
    return (var < CHASSIS_IMU_STILL_VAR_TH_DPS2) ? 1U : 0U;
}

/** 调试用: 读取最近一次静止判别结果 (1=静止) */
uint8 chassis_imu_is_still(void)
{
    return s_imu_is_still;
}

/**
 * @brief  初始化 IMU 硬件并执行静态零偏标定
 * @note   标定期间车体必须静止，否则会把运动角速度误认为零偏。
 */
void chassis_imu_init(void)
{
    float gyro_z_sum_dps = 0.0f; /* 标定窗口内角速度累计值 */
    uint16 sample_count = 1000U; /* 标定采样次数 */
    uint16 i;                    /* 采样循环计数 */

    /* 步骤 1: 初始化底层 IMU 设备。 */
    imu660rb_init();

    /* 步骤 2: 在静止状态采样偏航轴角速度，估计零偏。 */
    for (i = 0U; i < sample_count; ++i)
    {
        imu660rb_get_gyro();
        /* P0-修复 2026-04-29 IMU 装反: 原代码仅采 Z 轴, 现按轴选择宏切换
         *   gyro_z_sum_dps += imu660rb_gyro_transition(imu660rb_gyro_z);
         */
        gyro_z_sum_dps += imu_yaw_gyro_raw_dps();
        system_delay_ms(1);
    }

    /* 步骤 3: 计算并保存平均零偏。 */
    s_gyro_z_bias_dps = gyro_z_sum_dps / (float)sample_count;

    /* 步骤 4: 复位航向积分状态。
     * P0-修复 2026-04-29 (0.5° 零偏 bug):
     * 原本这里硬写 car_angle.yaw = CHASSIS_IMU_YAW_INIT_OFFSET_DEG (-0.5°),
     * 实际会造成“上电就偏 -0.5°”的静态偏差, 与 bias 标定重复补偿。
     * 现改为从 0 起, 依靠 bias 标定 + 滑窗辨识保证静止不漂. */
    car_angle.roll = 0.0f;
    car_angle.pitch = 0.0f;
    car_angle.yaw = 0.0f;
    s_yaw_rate_lpf_dps = 0.0f;
    s_yaw_rate_for_d_dps = 0.0f;
}

/**
 * @brief  外部设置航向角（用于重定位校正）
 * @param  yaw_deg 目标航向角（度）
 */
void chassis_imu_set_yaw_deg(float yaw_deg)
{
    car_angle.yaw = chassis_normalize_angle_deg(yaw_deg);
}

/**
 * @brief  读取当前航向角
 * @return 航向角（度，范围 [-180, 180]）
 */
float chassis_imu_get_yaw_deg(void)
{
    return car_angle.yaw;
}

/**
 * @brief  5ms 周期更新航向角
 * @note   处理链路：零偏补偿 -> 死区抑噪 -> 零偏自适应 -> 低通 -> 欧拉积分。
 */
void chassis_imu_update_5ms(void)
{
    float gyro_z_raw_dps;  /* 偏航轴原始角速度（度/秒） - 实际轴由 CHASSIS_IMU_YAW_AXIS 选择 */
    float yaw_rate_dps;    /* 零偏补偿与符号修正后的角速度（度/秒） */
    float still_mean_dps;  /* 滑窗均值 (度/秒)   - 静止时即为最佳 bias 估计 */
    float still_var_dps2;  /* 滑窗方差 (度/秒)^2 - 用于静止判别 */
    uint8 is_still;        /* 1 = 当前窗口认定静止 */

    /* 步骤 1: 采样底层传感器数据。 */
    imu660rb_get_acc();
    imu660rb_get_gyro();
    /* P0-修复 2026-04-29 IMU 装反: 原代码仅采 Z 轴, 现按轴选择宏切换
     *   gyro_z_raw_dps = imu660rb_gyro_transition(imu660rb_gyro_z);
     */
    gyro_z_raw_dps = imu_yaw_gyro_raw_dps();

    /* 步骤 2: 滑窗静止检测 -> 通用零偏在线辨识 (P0-改进 2026-04-29) */
    is_still = still_detect_step(gyro_z_raw_dps, &still_mean_dps, &still_var_dps2);
    s_imu_is_still = is_still;
    if (is_still)
    {
        /* 真静止: 用窗口均值快速 EMA 校准 bias (50ms 时间常数), 这是当前角速度的最佳零偏估计 */
        s_gyro_z_bias_dps += CHASSIS_IMU_BIAS_FAST_ALPHA * (still_mean_dps - s_gyro_z_bias_dps);
    }

    /* 步骤 3: 做零偏补偿与安装方向修正。 */
    yaw_rate_dps = (gyro_z_raw_dps - s_gyro_z_bias_dps) * IMU_YAW_SIGN;

    /* 步骤 3.5: D 项专用通道 (P0-修复 2026-04-29 段段 + 静止自走 bug)
     * 软死区 (soft-deadzone): r' = sign(r) * max(|r|-d, 0)
     *   - 边界处 r'=0 且斜率连续 -> KD*r' 不会跳变 -> 不会段段
     *   - 静止噪声 (|r|<0.8 dps) 后 r'=0 -> D=0 -> 不会被 KD=300 放大拉走车
     * 原因: 上一版完全去死区, KD=300 * 噪声 0.5 dps = 150 dps 虚假 wz 指令。 */
    {
        float r_abs = fabsf(yaw_rate_dps);
        float r_for_d;
        if (r_abs <= IMU_GYRO_DEADZONE_DPS)
        {
            r_for_d = 0.0f;
        }
        else
        {
            r_for_d = (yaw_rate_dps >= 0.0f) ? (r_abs - IMU_GYRO_DEADZONE_DPS)
                                             : -(r_abs - IMU_GYRO_DEADZONE_DPS);
        }
        s_yaw_rate_for_d_dps += IMU_GYRO_LPF_ALPHA * (r_for_d - s_yaw_rate_for_d_dps);
    }

    /* 步骤 4: 小角速度硬死区 (yaw 积分通道专用, 不影响上面的 D 通道) */
    if (fabsf(yaw_rate_dps) < IMU_GYRO_DEADZONE_DPS)
    {
        yaw_rate_dps = 0.0f;
    }

    /*
     * 步骤 5: 兜底慢通道 - 当滑窗判定运动但本拍角速度仍在死区内 (例如长时间
     * 缓慢漂移, 滑窗方差略超阈值, 但单拍速率几乎为 0), 用很小的 ALPHA 把
     * bias 慢慢拉过去, 防止温漂积累。
     *   原算法只有这一条通道, 现已弱化为兜底。
     */
    if ((0U == is_still) && (0.0f == yaw_rate_dps))
    {
        s_gyro_z_bias_dps += IMU_BIAS_ADAPT_ALPHA * (gyro_z_raw_dps - s_gyro_z_bias_dps);
    }

    /* 步骤 6: 一阶低通滤波，降低角速度噪声。 */
    s_yaw_rate_lpf_dps += IMU_GYRO_LPF_ALPHA * (yaw_rate_dps - s_yaw_rate_lpf_dps);

    /* 步骤 7: 欧拉积分并归一化到 [-180, 180]。 */
    car_angle.yaw += s_yaw_rate_lpf_dps * IMU_DT_S;
    car_angle.yaw = chassis_normalize_angle_deg(car_angle.yaw);
}

/* P0-修复 2026-04-29 段段 bug: 改为返回“无死区”D 项专用通道,
 * 避免 KD * yaw_rate 在死区边界突跳造成 wz_pi 输出段段。 */
float chassis_imu_get_yaw_rate_dps(void)
{
    return s_yaw_rate_for_d_dps;
}
