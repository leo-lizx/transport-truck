#ifndef CONFIG_SERVO_H
#define CONFIG_SERVO_H

/* BDS300 旋转舵机台架测试参数。
 * 脉宽映射统一采用官方例程 E02_06_servo_control_demo 的公式：
 *   duty = PWM_DUTY_MAX / (1000/freq) * (0.5 + angle/90)
 * 即 0.5ms→0°、1.5ms→90°、2.5ms→180°，比 BDS300 资料的 1–2ms 更宽。
 * 当前扫动 75–105° 约 1.33–1.67ms，仍在安全区间；若扩到 0–180°，
 * 两端会超出资料范围，必须空载确认。
 * 频率 300Hz：型号里的 “300” 是产品编号；300Hz 为用户要求且已实测可驱动。 */
#define SERVO_PWM_FREQ_HZ           (300U)
#define SERVO_ANGLE_RANGE_DEG       (180.0f)

/* 台架扫动范围：沿用官方例程的 75–105°，机构装好后按实际行程调整。 */
#define SERVO_TEST_START_DEG        (90.0f)
#define SERVO_TEST_MIN_DEG          (75.0f)
#define SERVO_TEST_MAX_DEG          (105.0f)
#define SERVO_TEST_STEP_DEG         (1.0f)
#define SERVO_TEST_STEP_MS          (50U)

#endif /* CONFIG_SERVO_H */
