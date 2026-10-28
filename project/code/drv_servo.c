/*
 * FeETech FS5323M 数字舵机 PWM 输出：P16 / C30，PWM4 子模块 2 A。
 * 脉宽采用官方例程公式：0.5ms→0°、1.5ms→90°、2.5ms→180°，
 * 与舵机 500–2500µs 的额定量程一致。
 * 模式 5 扫动 75–105°；模式 6 使用 0–180°，首次必须空载确认端点。
 */
#include "drv_servo.h"

#include "config/configServo.h"
#include "config/pinMap.h"
#include "zf_driver_pwm.h"

#if (SERVO_PWM_FREQ_HZ < 50U) || (SERVO_PWM_FREQ_HZ > 300U)
#error "SERVO_PWM_FREQ_HZ must stay in 50..300 Hz"
#endif

static float servo_clamp_deg(float deg)
{
    if (deg < 0.0f)
    {
        return 0.0f;
    }
    if (deg > SERVO_ANGLE_RANGE_DEG)
    {
        return SERVO_ANGLE_RANGE_DEG;
    }
    return deg;
}

/* 官方例程公式；PWM_DUTY_MAX 对应一个完整周期。 */
static float servo_duty_from_angle(float deg)
{
    return (float)PWM_DUTY_MAX / (1000.0f / (float)SERVO_PWM_FREQ_HZ) *
           (0.5f + deg / 90.0f);
}

void servo_set_angle(float deg)
{
    deg = servo_clamp_deg(deg);
    pwm_set_duty(SERVO_TEST_PWM_CHANNEL, (uint32)servo_duty_from_angle(deg));
}

void servo_init(float home_deg)
{
    pwm_init(SERVO_TEST_PWM_CHANNEL, SERVO_PWM_FREQ_HZ, 0U);
    servo_set_angle(home_deg);
}
