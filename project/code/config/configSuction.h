#ifndef CONFIG_SUCTION_H
#define CONFIG_SUCTION_H

/* 复现吸盘 PDF 的 Arduino Servo 默认示例，不代表开关额定规格已实测。
 * 来源：https://github.com/arduino-libraries/Servo/blob/master/src/Servo.h */
#define SUCTION_PWM_FREQ_HZ          (50U)
#define SUCTION_SWITCH_OFF_US        (544U)
#define SUCTION_SWITCH_ON_US         (2400U)

#define SUCTION_TEST_SAMPLE_MS       (5U)
#define SUCTION_TEST_DEBOUNCE_MS     (20U)
#define SUCTION_TEST_RELEASE_MS      (2000U)

#endif /* CONFIG_SUCTION_H */
