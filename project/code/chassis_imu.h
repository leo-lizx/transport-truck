#ifndef _CHASSIS_IMU_H_
#define _CHASSIS_IMU_H_

#include "zf_common_typedef.h"

// 姿态角结构体
typedef struct {
    float roll;      // 横滚角 (度)
    float pitch;     // 俯仰角 (度)
    float yaw;       // 偏航角 (度) - 推箱子最重要的角度
} EulerAngle_t;

// 外部引用的姿态角变量
extern EulerAngle_t car_angle;

// 函数声明
void IMU_Process_Init(void);         // IMU算法初始化（包含零偏采集，调用前需确保车模静止）
void IMU_Process_Update(void);       // IMU数据更新与姿态解算（放入 5ms 定时器中断中调用）
void IMU_Reset_Yaw(float new_yaw);   // 重置/校准 Yaw 角（用于视觉定期校正绝对角度）

#endif