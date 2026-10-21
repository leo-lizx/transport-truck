#ifndef APP_SERVO_TEST_H
#define APP_SERVO_TEST_H

#include <stdint.h>

/* 初始化舵机 PWM 并停在起始角。 */
void app_servo_test_init(void);

/* 主循环每 5ms 调用，now_ms 为累计节拍时间，允许 uint32_t 回绕。
 * 按 SERVO_TEST_STEP_MS 在 MIN/MAX 之间往返扫动。 */
void app_servo_test_task(uint32_t now_ms);

#endif /* APP_SERVO_TEST_H */
