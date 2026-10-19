/*
 * 吸盘两路电子开关：P17/D0 为泵，P18/D1 为阀。
 * 复现资料 Arduino Servo 示例的名义波形；不代表开关额定规格已确认。
 */
#include "drv_suction.h"

#include "config/configSuction.h"
#include "config/pinMap.h"
#include "fsl_clock.h"
#include "fsl_iomuxc.h"
#include "fsl_pwm.h"
#include "zf_common_debug.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"

#if (SUCTION_PWM_FREQ_HZ == 0U)
#error "SUCTION_PWM_FREQ_HZ must be nonzero"
#endif

#if (SUCTION_SWITCH_OFF_US == 0U) || \
    (SUCTION_SWITCH_OFF_US >= (1000000U / SUCTION_PWM_FREQ_HZ)) || \
    (SUCTION_SWITCH_ON_US == 0U) || \
    (SUCTION_SWITCH_ON_US >= (1000000U / SUCTION_PWM_FREQ_HZ))
#error "Suction switch pulses must be nonzero and shorter than one frame"
#endif

/* SDK 高精度占空比范围为 0..65535，保留微秒脉宽而非整数百分比。 */
static uint16_t suction_pulse_duty(uint32_t pulse_us)
{
    return (uint16_t)(((uint64_t)pulse_us * SUCTION_PWM_FREQ_HZ * UINT16_MAX +
                       500000U) / 1000000U);
}

void suction_set(uint8_t pump_on, uint8_t valve_on)
{
    uint16_t pump_duty = suction_pulse_duty((pump_on != 0U) ?
                                          SUCTION_SWITCH_ON_US : SUCTION_SWITCH_OFF_US);
    uint16_t valve_duty = suction_pulse_duty((valve_on != 0U) ?
                                           SUCTION_SWITCH_ON_US : SUCTION_SWITCH_OFF_US);

    PWM_SetPwmLdok(PWM1, kPWM_Control_Module_3, false);
    if (pump_on != 0U)
    {
        /* (1,0) 吸附命令先写阀 OFF，再写泵 ON。 */
        PWM_UpdatePwmDutycycleHighAccuracy(PWM1, kPWM_Module_3, kPWM_PwmB,
                                          kPWM_EdgeAligned, valve_duty);
        PWM_UpdatePwmDutycycleHighAccuracy(PWM1, kPWM_Module_3, kPWM_PwmA,
                                          kPWM_EdgeAligned, pump_duty);
    }
    else
    {
        /* 释放命令先写泵 OFF，再写阀 ON；(0,0) 则将阀也置 OFF。 */
        PWM_UpdatePwmDutycycleHighAccuracy(PWM1, kPWM_Module_3, kPWM_PwmA,
                                          kPWM_EdgeAligned, pump_duty);
        PWM_UpdatePwmDutycycleHighAccuracy(PWM1, kPWM_Module_3, kPWM_PwmB,
                                          kPWM_EdgeAligned, valve_duty);
    }
    /* 两路在同一帧边界装载，写入次序不保证阀已先完成机械动作。 */
    PWM_SetPwmLdok(PWM1, kPWM_Control_Module_3, true);
}

void suction_init(void)
{
    pwm_config_t pwm_config;
    pwm_signal_param_t pwm_signals[2] = {0};
    uint32_t source_clock_hz = CLOCK_GetFreq(kCLOCK_IpgClk);
    uint32_t period_counts = source_clock_hz / SUCTION_PWM_FREQ_HZ;
    uint8_t prescale = 0U;
    status_t status;

    /* 此双通道实现仅拥有 PWM1_SM3；改引脚时不能静默改用其他资源。 */
    zf_assert(SUCTION_PUMP_PWM_CHANNEL == PWM1_MODULE3_CHA_D0);
    zf_assert(SUCTION_VALVE_PWM_CHANNEL == PWM1_MODULE3_CHB_D1);

    while ((period_counts > UINT16_MAX) && (prescale < 7U))
    {
        prescale++;
        period_counts = (source_clock_hz >> prescale) / SUCTION_PWM_FREQ_HZ;
    }
    if ((period_counts == 0U) || (period_counts > UINT16_MAX))
    {
        zf_assert(0);
        return;
    }

    afio_init(IOMUXC_GPIO_SD_B1_00_FLEXPWM1_PWMA03, 0U,
              SPEED_100MHZ | KEEPER_EN | DSE_R0_6);
    afio_init(IOMUXC_GPIO_SD_B1_01_FLEXPWM1_PWMB03, 0U,
              SPEED_100MHZ | KEEPER_EN | DSE_R0_6);

    PWM_GetDefaultConfig(&pwm_config);
    pwm_config.prescale = (pwm_clock_prescale_t)prescale;
    pwm_config.pairOperation = kPWM_Independent;
    pwm_config.reloadLogic = kPWM_ReloadPwmFullCycle;
    pwm_config.enableDebugMode = true;
    status = PWM_Init(PWM1, kPWM_Module_3, &pwm_config);
    if (status != kStatus_Success)
    {
        zf_assert(0);
        return;
    }

    pwm_signals[0].pwmChannel = kPWM_PwmA;
    pwm_signals[0].level = kPWM_HighTrue;
    pwm_signals[1].pwmChannel = kPWM_PwmB;
    pwm_signals[1].level = kPWM_HighTrue;

    /* 不分别调用 pwm_init()，以免第二路初始化重置同子模块的第一路。 */
    PWM_SetPwmLdok(PWM1, kPWM_Control_Module_3, false);
    status = PWM_SetupPwm(PWM1, kPWM_Module_3, pwm_signals, 2U,
                         kPWM_EdgeAligned, SUCTION_PWM_FREQ_HZ, source_clock_hz);
    if (status != kStatus_Success)
    {
        zf_assert(0);
        return;
    }
    PWM1->SM[kPWM_Module_3].DISMAP[0] = 0U;
    suction_set(0U, 0U);
    /* 计数启动前已写入两路有效 OFF 脉冲，零占空比不表示开关 OFF。 */
    PWM_StartTimer(PWM1, kPWM_Control_Module_3);
}
