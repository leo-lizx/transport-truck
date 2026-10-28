#ifndef CONFIG_SERVO_H
#define CONFIG_SERVO_H

/* FeETech FS5323M 数字舵机台架测试参数（2026-10-28 换用）。
 * 厂家参数：4.8–6V、500–2500µs（中位 1500µs）、180°(±5°)、
 * 堵转 21.5kg·cm / 1.5A@6V、空载 0.16s/60°、25T 输出轴。
 * 脉宽映射统一采用官方例程公式：
 *   duty = PWM_DUTY_MAX / (1000/freq) * (0.5 + angle/90)
 * 即 0.5ms→0°、1.5ms→90°、2.5ms→180°，与舵机 500–2500µs 量程一致，
 * 0–180° 全行程都在额定范围内。
 * 频率恢复 300Hz：这是上一版已验证可动的设置；改为 50Hz 后舵机不动，
 * 先回到 300Hz 隔离“频率 vs 舵机/接线”两个变量。 */
#define SERVO_PWM_FREQ_HZ           (300U)
#define SERVO_ANGLE_RANGE_DEG       (180.0f)

/* 台架扫动范围：沿用官方例程的 75–105°，机构装好后按实际行程调整。 */
#define SERVO_TEST_START_DEG        (90.0f)
#define SERVO_TEST_MIN_DEG          (75.0f)
#define SERVO_TEST_MAX_DEG          (105.0f)
#define SERVO_TEST_STEP_DEG         (1.0f)
#define SERVO_TEST_STEP_MS          (50U)

/* 模式 6：上电自动执行一个周期 90° → 0° → 180° → 90°，
 * 每次转动后停顿 1.5s；完成后停在 90° 不再动作，重新上电才再跑一次。 */
#define SERVO_SEQ_HOME_DEG          (90.0f)
#define SERVO_SEQ_LOW_DEG           (0.0f)
#define SERVO_SEQ_HIGH_DEG          (180.0f)
#define SERVO_SEQ_PAUSE_MS          (1500U)

#endif /* CONFIG_SERVO_H */
