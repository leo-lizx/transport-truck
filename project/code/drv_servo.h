#ifndef DRV_SERVO_H
#define DRV_SERVO_H

/* 初始化 P16/C30 舵机 PWM（50Hz），并停在测试起始角。 */
void servo_init(void);

/* 设置舵机角度（0–180°），超范围会被限幅。 */
void servo_set_angle(float deg);

#endif /* DRV_SERVO_H */
