#include "chassis_imu.h"
#include "zf_device_imu660rb.h"
#include "zf_driver_delay.h"
#include <math.h>

// 算法常量定义
#define DT              0.005f      // 积分步长 5ms
#define GYRO_DEADZONE   0.15f       // 陀螺仪死区 (度/秒)，滤除静止时的微小噪声

// Mahony 滤波参数
#define Kp 2.0f   // 比例增益，控制加速度计收敛到重力方向的速度
#define Ki 0.005f // 积分增益，控制陀螺仪零偏漂移的收敛

// 姿态输出
EulerAngle_t car_angle = {0};

// 内部状态变量
static float gyro_z_bias = 0.0f;    // Z轴陀螺仪静态零偏
static float last_gyro_z = 0.0f;    // 上一次的 Z 轴角速度（用于梯形积分）

// Mahony 四元数与积分误差
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

//-------------------------------------------------------------------------
// 函数简介：快速计算平方根的倒数 (RT1064有硬件FPU，直接用1.0f/sqrtf也可，这里保留经典算法)
//-------------------------------------------------------------------------
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//-------------------------------------------------------------------------
// 函数简介：IMU 初始化（包含静态零偏采集）
// 备注：调用此函数时，务必保证车模放在平地上且【绝对静止】！
//-------------------------------------------------------------------------
void chassis_imu_init(void) {
    float sum_gyro_z = 0.0f;
    int sample_count = 1000;  // 采集1000次求平均

    // 1. 底层硬件初始化
    imu660rb_init();
    
    // 2. 静态零偏标定
    for(int i = 0; i < sample_count; i++) {
        imu660rb_get_gyro(); // 底层读取
        sum_gyro_z += imu660rb_gyro_transition(imu660rb_gyro_z); // 转换为 °/s 并累加
        system_delay_ms(1);  // 短暂延时
    }
    
    // 3. 计算 Z 轴零偏
    gyro_z_bias = sum_gyro_z / (float)sample_count;
    
    // 4. 初始化状态
    car_angle.yaw = 0.0f;
    last_gyro_z = 0.0f;
}

//-------------------------------------------------------------------------
// 函数简介：设置/校准 Yaw 角（度）
//-------------------------------------------------------------------------
void chassis_imu_set_yaw_deg(float yaw_deg) {
    car_angle.yaw = yaw_deg;
}

float chassis_imu_get_yaw_deg(void) {
    return car_angle.yaw;
}

//-------------------------------------------------------------------------
// 函数简介：IMU数据更新与姿态解算 (5ms 高频调用)
// 备注：必须放在严谨的 5ms 定时器中断中执行！
//-------------------------------------------------------------------------
void chassis_imu_update_5ms(void) {
    float ax, ay, az;
    float gx, gy, gz;
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // 1. 获取底层原始数据并转换为物理单位
    imu660rb_get_acc();
    imu660rb_get_gyro();
    
    ax = imu660rb_acc_transition(imu660rb_acc_x); // 单位: g
    ay = imu660rb_acc_transition(imu660rb_acc_y);
    az = imu660rb_acc_transition(imu660rb_acc_z);
    
    gx = imu660rb_gyro_transition(imu660rb_gyro_x); // 单位: °/s
    gy = imu660rb_gyro_transition(imu660rb_gyro_y);
    gz = imu660rb_gyro_transition(imu660rb_gyro_z);

    // ==========================================================
    // 第一部分：高精度 Yaw 角解算 (死区 + 梯形积分)
    // ==========================================================
    float current_gz = gz - gyro_z_bias; // 消除静态零偏
    
    // 剔除静止死区噪声
    if(fabs(current_gz) < GYRO_DEADZONE) {
        current_gz = 0.0f;
    }
    
    // 梯形积分求偏航角：Yaw += (上一次角速度 + 本次角速度) * dt / 2
    car_angle.yaw += (last_gyro_z + current_gz) * DT / 2.0f;
    last_gyro_z = current_gz; // 更新历史值

    // 限制 Yaw 在 0~360 或 -180~180 之间 (根据个人习惯，这里做 -180~180 限制)
    if(car_angle.yaw > 180.0f)  car_angle.yaw -= 360.0f;
    if(car_angle.yaw < -180.0f) car_angle.yaw += 360.0f;


    // ==========================================================
    // 第二部分：Mahony 算法解算 Pitch 和 Roll (加速度计与陀螺仪融合)
    // ==========================================================
    // 将陀螺仪数据转换为 弧度/秒，供四元数运算使用
    gx = gx * 0.0174533f; 
    gy = gy * 0.0174533f;
    gz = gz * 0.0174533f;

    // 只在加速度计数据有效时进行修正
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // 归一化加速度计数据
        norm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= norm;
        ay *= norm;
        az *= norm;

        // 根据四元数计算出的当前重力在三个轴上的估计投影
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // 测量得到的重力向量与估计的重力向量之间的误差 (叉乘)
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // 误差积分
        exInt += ex * Ki;
        eyInt += ey * Ki;
        ezInt += ez * Ki;

        // 调整陀螺仪的测量值
        gx += Kp * ex + exInt;
        gy += Kp * ey + eyInt;
        gz += Kp * ez + ezInt;
    }

    // 整合四元数变化率并归一化 (一阶龙格库塔法)
    float q0_last = q0, q1_last = q1, q2_last = q2, q3_last = q3;
    q0 += (-q1_last * gx - q2_last * gy - q3_last * gz) * (0.5f * DT);
    q1 += ( q0_last * gx + q2_last * gz - q3_last * gy) * (0.5f * DT);
    q2 += ( q0_last * gy - q1_last * gz + q3_last * gx) * (0.5f * DT);
    q3 += ( q0_last * gz + q1_last * gy - q2_last * gx) * (0.5f * DT);

    // 四元数归一化
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    // 将四元数转换为欧拉角 (仅计算 Pitch 和 Roll)
    car_angle.pitch = asinf(-2.0f * q1 * q3 + 2.0f * q0 * q2) * 57.29578f;
    car_angle.roll  = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 57.29578f;
}
