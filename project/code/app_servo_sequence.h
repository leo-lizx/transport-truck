#ifndef APP_SERVO_SEQUENCE_H
#define APP_SERVO_SEQUENCE_H

#include <stdint.h>

/* 模式 6：上电自动执行一个周期 90° → 0° → 180° → 90°，
 * 每次转动后停顿 SERVO_SEQ_PAUSE_MS；完成后停在 90°，不再动作。 */
void app_servo_sequence_init(void);

/* 主循环每 5ms 调用，now_ms 允许 uint32_t 回绕；完成后直接返回。 */
void app_servo_sequence_task(uint32_t now_ms);

#endif /* APP_SERVO_SEQUENCE_H */
