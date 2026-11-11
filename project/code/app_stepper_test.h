#ifndef APP_STEPPER_TEST_H
#define APP_STEPPER_TEST_H

#include <stdint.h>

/* 模式 9：升降步进台架测试，底盘保持 STOP。
 * 进入主循环 1 s 后自动回原点（快找开关 → 退 2 mm → 慢碰定零 → 停在 2 mm）。
 * 就绪后：K1 远离开关 10 mm，K2 朝开关 10 mm（均限制在软件行程内），
 *         K3 重新回原点，K4 任何时候立即停止并进入停机状态。 */
typedef enum
{
    APP_STEPPER_WAIT_START = 0,
    APP_STEPPER_HOME_FAST,
    APP_STEPPER_HOME_BACKOFF,
    APP_STEPPER_HOME_SLOW,
    APP_STEPPER_TO_READY,
    APP_STEPPER_READY,
    APP_STEPPER_FAULT_NO_SWITCH,   /* 找开关走满最大行程仍未触发 */
    APP_STEPPER_STOPPED            /* K4 急停 */
} app_stepper_state_t;

void app_stepper_test_init(void);

/* 主循环每 5ms 调用，now_ms 允许 uint32_t 回绕。 */
void app_stepper_test_task(uint32_t now_ms);

/* 供 Live Watch / 主机测试观察当前状态。 */
app_stepper_state_t app_stepper_test_get_state(void);

#endif /* APP_STEPPER_TEST_H */
