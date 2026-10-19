#ifndef APP_SUCTION_TEST_H
#define APP_SUCTION_TEST_H

#include <stdint.h>

/* 初始化 C15 低有效按键和吸盘；上电必须稳定松手后才允许吸附。 */
void app_suction_test_init(void);

/* 主循环每 5ms 调用，now_ms 为累计节拍时间，允许 uint32_t 回绕。
 * 按住吸附；松手泄压 2 秒；泄压期间再次按下则恢复吸附。 */
void app_suction_test_task(uint32_t now_ms);

#endif /* APP_SUCTION_TEST_H */
