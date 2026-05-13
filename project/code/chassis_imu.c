/*===========================================================================
 * [chassis_imu.c] IMU 航向采样与积分模块实现
 *---------------------------------------------------------------------------
 *  分区索引:
 *    § 1. 底层硬件接口 (imu_yaw_gyro_raw_dps)              [L  20±]
 *    § 2. KF 状态与预测/ZUPT 更新  (kalman_predict/update)    [L  76±]
 *    § 3. 滑窗静止检测           (still_detect_step)            [L 163±]
 *    § 4. 初始化与零偏标定       (chassis_imu_init)             [L 211±]
 *    § 5. yaw 读/写 + 5ms 处理链 (chassis_imu_update_5ms)        [L 288±]
 *    § 6. KF 观测更新对外接口     (chassis_imu_kf_correct_angle) [L 420±]
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

/* ==========================================================================
 *  § 2. Yaw 卡尔曼滤波 (P0-改进 2026-05-02)
 * ========================================================================== */
/* ----------------------------------------------------------------------
 *  Yaw 卡尔曼滤波 (P0-改进 2026-05-02)
 *  状态: x = [angle, bias]ᵀ
 *  - kf_angle  : 当前 yaw 角 (°), 等价于车体 yaw (受 IMU_YAW_SIGN / 轴选择影响)
 *  - kf_bias   : gyro 零偏 (°/s), 在 dps 域, 与 s_gyro_z_bias_dps 物理含义一致
 *  - kf_P[2][2]: 状态协方差
 *  KF 在 5ms 节拍下做"预测 + (静止时)ZUPT 观测 + 角度归一化".
 * ---------------------------------------------------------------------- */
#if (CHASSIS_IMU_USE_KALMAN_YAW != 0)
static float s_kf_angle = 0.0f;
static float s_kf_bias  = 0.0f;
static float s_kf_P[2][2] = {
    { CHASSIS_IMU_KF_P0_ANGLE_DEG2, 0.0f },
    { 0.0f,                          CHASSIS_IMU_KF_P0_BIAS_DPS2 }
};

/**
 * @brief KF 预测步: 用 (gyro - bias)*dt 推进 angle, bias 不变, 协方差按
 *        F = [[1,-dt],[0,1]], Q = diag(Q_angle*dt, Q_bias*dt) 传播.
 *        F P Fᵀ + Q 展开后逐元素更新, 避免矩阵库依赖.
 * @param gyro_dps  本拍原始 gyro (已经做 IMU_YAW_SIGN 修正), 单位 °/s
 * @param dt        采样间隔 (s)
 */
static void kalman_predict(float gyro_dps, float dt)
{
    /* 1) 状态推进: angle += (gyro - bias) * dt; bias 不变 */
    float rate = gyro_dps - s_kf_bias;
    s_kf_angle += rate * dt;

    /* 2) 协方差推进 P = F P Fᵀ + Q
     *    F = [[1, -dt], [0, 1]]
     *    展开:
     *      P00' = P00 - dt*(P10+P01) + dt²*P11 + Q_angle*dt
     *      P01' = P01 - dt*P11
     *      P10' = P10 - dt*P11
     *      P11' = P11 + Q_bias*dt
     */
    float P00 = s_kf_P[0][0];
    float P01 = s_kf_P[0][1];
    float P10 = s_kf_P[1][0];
    float P11 = s_kf_P[1][1];

    s_kf_P[0][0] = P00 - dt * (P10 + P01) + dt * dt * P11
                       + CHASSIS_IMU_KF_Q_ANGLE_DPS2 * dt;
    s_kf_P[0][1] = P01 - dt * P11;
    s_kf_P[1][0] = P10 - dt * P11;
    s_kf_P[1][1] = P11 + CHASSIS_IMU_KF_Q_BIAS_DPS2_PER_S * dt;
}

/**
 * @brief KF 静止 ZUPT 观测步: 静止时认定真实角速度=0, 故 z = gyro 即对 bias 的观测.
 *        H = [0, 1], 标量 S = P11 + R, 增益 K = [P01/S, P11/S]ᵀ
 * @param gyro_dps 当前原始 gyro (°/s), 已做 IMU_YAW_SIGN 修正
 */
static void kalman_update_zupt(float gyro_dps)
{
    /* 1) 创新 y = z - H*x = gyro - bias */
    float y = gyro_dps - s_kf_bias;

    /* 2) 创新协方差 S = H P Hᵀ + R = P11 + R */
    float S = s_kf_P[1][1] + CHASSIS_IMU_KF_R_ZUPT_DPS2;
    if (S < 1e-9f) { S = 1e-9f; }       /* 数值兜底, 防 0 除 */

    /* 3) 卡尔曼增益 K = P Hᵀ / S = [P01/S, P11/S]ᵀ */
    float K0 = s_kf_P[0][1] / S;        /* 角度增益: ZUPT 主要修 bias, 这一项很小 */
    float K1 = s_kf_P[1][1] / S;        /* bias 增益: 主战场 */

    /* 4) 状态更新 x = x + K y */
    s_kf_angle += K0 * y;
    s_kf_bias  += K1 * y;

    /* 5) 协方差更新 P = (I - K H) P, H=[0,1] -> KH = [[0,K0],[0,K1]]
     *      P00' = P00 - K0*P10
     *      P01' = P01 - K0*P11
     *      P10' = (1-K1)*P10
     *      P11' = (1-K1)*P11
     */
    float P00 = s_kf_P[0][0];
    float P01 = s_kf_P[0][1];
    float P10 = s_kf_P[1][0];
    float P11 = s_kf_P[1][1];

    s_kf_P[0][0] = P00 - K0 * P10;
    s_kf_P[0][1] = P01 - K0 * P11;
    s_kf_P[1][0] = (1.0f - K1) * P10;
    s_kf_P[1][1] = (1.0f - K1) * P11;
}
#endif /* CHASSIS_IMU_USE_KALMAN_YAW */
/* ==========================================================================
 *  § 3. 滑窗静止检测 (与 KF ZUPT 联动)
 * ========================================================================== */
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

/* ==========================================================================
 *  § 4. 初始化与静态零偏标定 (启动期唯一调用一次)
 * ========================================================================== */

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

#if (CHASSIS_IMU_USE_KALMAN_YAW != 0)
    /* 步骤 5: KF 状态初值. bias 直接吃掉刚刚 1000 次平均的零偏估计;
     *         angle = 0; P 用 config 的初值 (上面静态初始化已设过, 这里
     *         无条件重置一次, 防热复位时残留). */
    s_kf_angle = 0.0f;
    s_kf_bias  = s_gyro_z_bias_dps;
    s_kf_P[0][0] = CHASSIS_IMU_KF_P0_ANGLE_DEG2;
    s_kf_P[0][1] = 0.0f;
    s_kf_P[1][0] = 0.0f;
    s_kf_P[1][1] = CHASSIS_IMU_KF_P0_BIAS_DPS2;
#endif
}

/**
 * @brief  外部设置航向角（用于重定位校正）
 * @param  yaw_deg 目标航向角（度）
 */
void chassis_imu_set_yaw_deg(float yaw_deg)
{
    car_angle.yaw = chassis_normalize_angle_deg(yaw_deg);
#if (CHASSIS_IMU_USE_KALMAN_YAW != 0)
    /* 同步 KF 状态, 避免外部重定位后 KF 自行回拉 */
    s_kf_angle = car_angle.yaw;
    /* 重定位说明角度强制可信, 角度方差降到很小 (1°²->0.01°²), 但 bias 不动 */
    s_kf_P[0][0] = 0.01f;
    s_kf_P[0][1] = 0.0f;
    s_kf_P[1][0] = 0.0f;
#endif
}

/**
 * @brief  读取当前航向角
 * @return 航向角（度，范围 [-180, 180]）
 */
float chassis_imu_get_yaw_deg(void)
{
    return car_angle.yaw;
}

/* ==========================================================================
 *  § 5. 5ms 处理链 (主调用点: 底盘 5ms PIT_IRQ)
 * ========================================================================== */

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
#if (CHASSIS_IMU_USE_KALMAN_YAW == 0)
    if (is_still)
    {
        /* 真静止: 用窗口均值快速 EMA 校准 bias (50ms 时间常数), 这是当前角速度的最佳零偏估计 */
        s_gyro_z_bias_dps += CHASSIS_IMU_BIAS_FAST_ALPHA * (still_mean_dps - s_gyro_z_bias_dps);
    }
#else
    /* KF 路径: bias 由下面的 ZUPT 自适应增益更新; 这里不再 EMA */
    (void)still_mean_dps;
    (void)still_var_dps2;
#endif

    /* 步骤 3: 做零偏补偿与安装方向修正。
     * KF 模式下 bias 由 KF 维护, 这里 yaw_rate_dps 仅用于 D 项软死区通道
     * 和兜底慢通道, 不直接进 yaw 积分. */
#if (CHASSIS_IMU_USE_KALMAN_YAW != 0)
    /* KF 路径: 用 KF 自身的 bias, 保持 D 通道 / 兜底逻辑物理含义不变 */
    yaw_rate_dps = (gyro_z_raw_dps - s_kf_bias) * IMU_YAW_SIGN;
#else
    yaw_rate_dps = (gyro_z_raw_dps - s_gyro_z_bias_dps) * IMU_YAW_SIGN;
#endif

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
     *   原算法只有这一条通道, 现已弱化为兜底; KF 模式下完全交给 KF, 跳过. */
#if (CHASSIS_IMU_USE_KALMAN_YAW == 0)
    if ((0U == is_still) && (0.0f == yaw_rate_dps))
    {
        s_gyro_z_bias_dps += IMU_BIAS_ADAPT_ALPHA * (gyro_z_raw_dps - s_gyro_z_bias_dps);
    }
#endif

    /* 步骤 6: 一阶低通滤波，降低角速度噪声。 (D 通道仍依赖该 LPF 的输出?
     * 实际 D 通道用的是 s_yaw_rate_for_d_dps, 这里 s_yaw_rate_lpf_dps 仅供
     * 旧路径积分使用; KF 路径下保留它给外部 get_yaw_rate_filtered 兼容.) */
    s_yaw_rate_lpf_dps += IMU_GYRO_LPF_ALPHA * (yaw_rate_dps - s_yaw_rate_lpf_dps);

    /* 步骤 7: yaw 角更新.
     *   旧路径: 简单欧拉积分;
     *   KF 路径: 调 KF 预测 (用未做硬死区的原始 gyro_signed), 静止时 ZUPT,
     *            把 KF 输出写回 car_angle.yaw. 这样 bias 可以越用越准. */
#if (CHASSIS_IMU_USE_KALMAN_YAW != 0)
    {
        /* 给 KF 喂"已做安装方向修正、未减 bias、未做死区"的 gyro 值,
         * KF 内部用自己的 s_kf_bias 做减法, 才能真正学到 bias. */
        float gyro_signed_dps = gyro_z_raw_dps * IMU_YAW_SIGN;

        kalman_predict(gyro_signed_dps, IMU_DT_S);

        /* 静止时做 ZUPT 观测: 真实角速度=0, 故 z=gyro 是对 bias 的观测 */
        if (is_still)
        {
            kalman_update_zupt(gyro_signed_dps);
        }

        /* 角度归一化到 [-180,180], 同步写回 car_angle 与外部用的 bias 镜像 */
        s_kf_angle = chassis_normalize_angle_deg(s_kf_angle);
        car_angle.yaw = s_kf_angle;
        s_gyro_z_bias_dps = s_kf_bias;     /* 让旧的 bias 调试访问仍可用 */
    }
#else
    car_angle.yaw += s_yaw_rate_lpf_dps * IMU_DT_S;
    car_angle.yaw = chassis_normalize_angle_deg(car_angle.yaw);
#endif
}

/* P0-修复 2026-04-29 段段 bug: 改为返回“无死区”D 项专用通道,
 * 避免 KD * yaw_rate 在死区边界突跳造成 wz_pi 输出段段。 */
float chassis_imu_get_yaw_rate_dps(void)
{
    return s_yaw_rate_for_d_dps;
}

/* ----------------------------------------------------------------------
 * KF 角度量测更新 (P0-改进 2026-05-02 编码器融合):
 * 给定外部观测 z = yaw_obs_deg, H=[1,0], 标量 KF 更新:
 *   y = z - x[0]                         (创新, 已做最近角归一)
 *   S = P00 + R                          (创新协方差)
 *   K = [P00/S, P10/S]ᵀ                  (增益)
 *   x += K*y;  P = (I - K H) P
 * 注意:
 *   - R 由调用方决定; odom 用大 R (e.g. 100), GPS/视觉可用小 R.
 *   - 创新做"差值再归一化"防止 ±180° 跨越时拉错方向.
 *   - 仅 KF 模式下生效, 旧路径下退化为 set_yaw_deg 行为以保兼容.
 * ---------------------------------------------------------------------- */
void chassis_imu_kf_correct_angle(float yaw_obs_deg, float R_deg2)
{
#if (CHASSIS_IMU_USE_KALMAN_YAW != 0)
    /* 1) 创新: 把差值归一化到 [-180,180], 防止 179° -> -179° 跨越拉错方向 */
    float y = chassis_normalize_angle_deg(yaw_obs_deg - s_kf_angle);

    /* 2) S = P00 + R, R 兜底防 0 */
    float S = s_kf_P[0][0] + (R_deg2 > 1e-6f ? R_deg2 : 1e-6f);

    /* 3) 增益 */
    float K0 = s_kf_P[0][0] / S;
    float K1 = s_kf_P[1][0] / S;

    /* 4) 状态更新 */
    s_kf_angle += K0 * y;
    s_kf_bias  += K1 * y;
    s_kf_angle = chassis_normalize_angle_deg(s_kf_angle);

    /* 5) 协方差更新 P = (I - KH) P, H=[1,0] -> KH=[[K0,0],[K1,0]]
     *      P00' = (1-K0)*P00
     *      P01' = (1-K0)*P01
     *      P10' = P10 - K1*P00
     *      P11' = P11 - K1*P01
     */
    float P00 = s_kf_P[0][0];
    float P01 = s_kf_P[0][1];
    float P10 = s_kf_P[1][0];
    float P11 = s_kf_P[1][1];
    s_kf_P[0][0] = (1.0f - K0) * P00;
    s_kf_P[0][1] = (1.0f - K0) * P01;
    s_kf_P[1][0] = P10 - K1 * P00;
    s_kf_P[1][1] = P11 - K1 * P01;

    /* 6) 同步对外可见的 yaw */
    car_angle.yaw = s_kf_angle;
#else
    /* 非 KF 路径: 弱化为 set_yaw_deg 半融合 — 取 90% 旧 + 10% 新 */
    float y = chassis_normalize_angle_deg(yaw_obs_deg - car_angle.yaw);
    car_angle.yaw = chassis_normalize_angle_deg(car_angle.yaw + 0.10f * y);
    (void)R_deg2;
#endif
}
