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

/* 底盘稳定调用接口（统一命名） */
void chassis_imu_init(void);
void chassis_imu_update_5ms(void);
float chassis_imu_get_yaw_deg(void);
void chassis_imu_set_yaw_deg(float yaw_deg);

#endif
