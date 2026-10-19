#ifndef DRV_SUCTION_H
#define DRV_SUCTION_H

#include <stdint.h>

/* 初始化 P17/D0 泵开关和 P18/D1 阀开关，两路持续发送 OFF 脉冲。 */
void suction_init(void);

/* 初始化后在主循环调用；非零表示 ON，零表示 OFF，不包含气动延时。 */
void suction_set(uint8_t pump_on, uint8_t valve_on);

#endif /* DRV_SUCTION_H */
