#ifndef CONFIG_STEPPER_H
#define CONFIG_STEPPER_H

/* 升降步进台架参数：RMN28 脉冲/方向驱动器 + FSK30 丝杆模组。
 * 已确认：导程 12 mm（手转 10 圈实测）、拨码 1/8 细分、限位开关按下导通。
 * 待确认：电机步距角按 1.8°（200 步/圈）处理，须核对铭牌；
 *         有效行程未测，软件上限先取保守值，实测后再放宽。
 * 坐标：0 = 限位开关触发点，正方向 = 远离限位开关。 */
#define STEPPER_FULL_STEPS_PER_REV      (200U)
#define STEPPER_MICROSTEP               (8U)     /* S1=ON S2=OFF S3=OFF */
#define STEPPER_LEAD_MM                 (12.0f)
#define STEPPER_STEPS_PER_MM            ((float)(STEPPER_FULL_STEPS_PER_REV * STEPPER_MICROSTEP) \
                                         / STEPPER_LEAD_MM)

/* 驱动器输入脉冲范围 100 Hz–20 kHz（厂家参数）；驱动层按此钳位。 */
#define STEPPER_DRIVER_MIN_HZ           (100.0f)
#define STEPPER_DRIVER_MAX_HZ           (20000.0f)

/* 首测速度取低值：起停 2 mm/s（约 267 Hz），点动 10 mm/s（约 1333 Hz）。
 * 加速度 100 mm/s² 远低于模组额定 600 mm/s²；不丢步后再逐步提高。 */
#define STEPPER_START_SPEED_MMPS        (2.0f)
#define STEPPER_JOG_SPEED_MMPS          (10.0f)
#define STEPPER_ACCEL_MMPS2             (100.0f)

/* 回原点：快速找开关 → 退出 → 慢速再碰一次定零点 → 停到就绪位置。 */
#define STEPPER_HOME_FAST_MMPS          (5.0f)
#define STEPPER_HOME_SLOW_MMPS          (2.0f)
#define STEPPER_HOME_BACKOFF_MM         (2.0f)
#define STEPPER_HOME_SLOW_SEARCH_MM     (4.0f)   /* 慢速再碰的最大行程 */
#define STEPPER_READY_MM                (2.0f)   /* 回零后停在离开关 2 mm 处 */

/* 软件行程：有效行程实测前保守取 40 mm；找开关最多走 上限 + 10 mm。 */
#define STEPPER_SOFT_MIN_MM             (1.0f)
#define STEPPER_SOFT_MAX_MM             (40.0f)
#define STEPPER_HOME_SEARCH_MM          (STEPPER_SOFT_MAX_MM + 10.0f)

/* 每按一次 K1/K2 点动的距离。 */
#define STEPPER_JOG_MM                  (10.0f)

/* D13 方向电平：首测确认。若回原点时滑台远离开关，立即按 K4 停止，
 * 把此值改为 0U（或互换 A+/A-）后重新编译。 */
#define STEPPER_DIR_LEVEL_AWAY_FROM_HOME (1U)

/* 限位开关：按下导通到 GND，D4 上拉输入，低电平 = 触发。
 * 朝开关方向运动时，连续 N 个步进周期读到触发才停，滤掉单次毛刺。 */
#define STEPPER_HOME_DEBOUNCE_STEPS     (3U)

/* 主循环节拍与按键去抖。 */
#define STEPPER_TEST_TASK_PERIOD_MS     (5U)
#define STEPPER_TEST_KEY_DEBOUNCE_MS    (20U)
#define STEPPER_TEST_START_DELAY_MS     (1000U)  /* 进入主循环后等 1 s 再回原点 */

#endif /* CONFIG_STEPPER_H */
