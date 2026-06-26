#ifndef _PIN_MAP_H_
#define _PIN_MAP_H_

/*
 * RT1064 主控 硬件映射头(pinMap) — 引脚单一真相源
 *
 * 集中原散落在 chassis_config.h 中的引脚定义。零依赖纯 #define:
 * 不 #include 任何驱动头；消费 .c 各自 include 对应驱动头展开 token
 * (PWM/ADC/GPIO 枚举值由各驱动头提供，本文件不引入)，保证本头
 * 无编译依赖、可被任意模块安全 include。
 *
 * 命名约定: {功能}_{通道/轮/实例}_{类型}
 *   例: MOTOR_LF_PWM_CHANNEL / ENCODER_LF_CH1 / UART_OPENART_TX
 *
 * 非引脚的频率/步长/时序/增益参数不在此(在 configChassis.h 或各模块本地)。
 *
 * 极性/方向系数**不在本文件**: 它们属"参数"而非"引脚",
 * 集中在 configChassis.h 的 §极性段 (执行器 *_OUTPUT_DIR / 编码器 ENCODER_*_DIR /
 * IMU *_SIGN)。本文件改动后须同步跨核引脚冲突矩阵 docs/PINOUT
 * (先改本头再更新矩阵)。
 */

/* ============================================================
 * 执行器驱动: 电机 PWM + 方向 GPIO
 *   DRV8701E 双路驱动，四轮独立控制。PWM 载波 17kHz。
 *   车轮编号 (俯视图，车头朝上):
 *          ┌── 车头(前方)──┐
 *          │  LF        RF  │   LF=左前  RF=右前
 *          │  LB        RB  │   LB=左后  RB=右后
 *          └────────────────┘
 * ============================================================ */

/* ---- 左前轮 LF (MOTOR4: PWM=C11, DIR=C10) ---- */
#define MOTOR_LF_PWM_CHANNEL    PWM2_MODULE2_CHB_C11
#define MOTOR_LF_DIR_PIN        C10

/* ---- 右前轮 RF (MOTOR3: PWM=D3, DIR=D2) 实车接线已验证 ---- */
#define MOTOR_RF_PWM_CHANNEL    PWM2_MODULE3_CHB_D3
#define MOTOR_RF_DIR_PIN        D2

/* ---- 左后轮 LB (MOTOR2: PWM=C6, DIR=C7) ---- */
#define MOTOR_LB_PWM_CHANNEL    PWM2_MODULE0_CHA_C6
#define MOTOR_LB_DIR_PIN        C7

/* ---- 右后轮 RB (MOTOR1: PWM=C8, DIR=C9) 实车接线已验证 ---- */
#define MOTOR_RB_PWM_CHANNEL    PWM2_MODULE1_CHA_C8
#define MOTOR_RB_DIR_PIN        C9

/* ============================================================
 * 反馈传感器: 正交编码器 (QTIMER)
 *
 * 平台: RT1064 学习主板，编码器接口定义:
 *   ENCODER_1 = QTIMER1_ENCODER1, A:C0  B:C1
 *   ENCODER_2 = QTIMER1_ENCODER2, A:C2  B:C24
 *   ENCODER_3 = QTIMER2_ENCODER1, A:C3  B:C4
 *   ENCODER_4 = QTIMER2_ENCODER2, A:C5  B:C25
 *
 * 轮子与编码器对应关系 (wfb 调试输出实测标定):
 *   左前轮 LF → ENCODER_4 (QTIMER2_ENCODER2)
 *   右前轮 RF → ENCODER_3 (QTIMER2_ENCODER1)
 *   左后轮 LB → ENCODER_1 (QTIMER1_ENCODER1)
 *   右后轮 RB → ENCODER_2 (QTIMER1_ENCODER2)
 *
 * !!! 更换电机/编码器线序/减速箱/轮位后必须重新标定 !!!
 * ============================================================ */

/* ---- 左前轮 LF → ENCODER_4 (QTIMER2_ENCODER2) ---- */
#define ENCODER_LF_INDEX        QTIMER2_ENCODER2
#define ENCODER_LF_CH1          QTIMER2_ENCODER2_CH1_C5
#define ENCODER_LF_CH2          QTIMER2_ENCODER2_CH2_C25

/* ---- 右前轮 RF → ENCODER_3 (QTIMER2_ENCODER1) ---- */
#define ENCODER_RF_INDEX        QTIMER2_ENCODER1
#define ENCODER_RF_CH1          QTIMER2_ENCODER1_CH1_C3
#define ENCODER_RF_CH2          QTIMER2_ENCODER1_CH2_C4

/* ---- 左后轮 LB → ENCODER_1 (QTIMER1_ENCODER1) ---- */
#define ENCODER_LB_INDEX        QTIMER1_ENCODER1
#define ENCODER_LB_CH1          QTIMER1_ENCODER1_CH1_C0
#define ENCODER_LB_CH2          QTIMER1_ENCODER1_CH2_C1

/* ---- 右后轮 RB → ENCODER_2 (QTIMER1_ENCODER2) ---- */
#define ENCODER_RB_INDEX        QTIMER1_ENCODER2
#define ENCODER_RB_CH1          QTIMER1_ENCODER2_CH1_C2
#define ENCODER_RB_CH2          QTIMER1_ENCODER2_CH2_C24

/* 编码器通道反查 (用于里程计/运动学，与上方轮位映射互逆):
 *   ENCODER_1 (QTIMER1_ENCODER1) → 左后轮 LB
 *   ENCODER_2 (QTIMER1_ENCODER2) → 右后轮 RB
 *   ENCODER_3 (QTIMER2_ENCODER1) → 右前轮 RF
 *   ENCODER_4 (QTIMER2_ENCODER2) → 左前轮 LF
 */

/* ============================================================
 * 人机接口: 按键 K1~K4 + 蜂鸣器 + IPS200
 *   按键丝印顺序可能与其它板反向 — 上板后核对
 * ============================================================ */
#define KEY_K1_PIN              C15
#define KEY_K2_PIN              C14
#define KEY_K3_PIN              C13
#define KEY_K4_PIN              C12

/* BEEP 引脚 — 待补充 (如使用) */
/* #define BEEP_PIN               XX */

/* IPS200 SPI — 经 zf_device_ips200 驱动，引脚见其初始化 */
/* IMU660RB SPI — 经 chassis_imu 驱动，引脚见其初始化 */

/* ============================================================
 * 通信收发器: OpenART 视觉 UART
 *   常用 UART4，默认 TX=C16 / RX=C17。
 *   若底板为 D0/D1，修改以下宏并在 configChassis.h 同步通信参数。
 *   Debug printf 可能共用同一物理口 — 接线时分清 MCU Debug TX。
 *
 *   方向控制 (如 RS-485 RE#/DE): 若使用半双工收发器，
 *   需补充 DIR 引脚并注明高低电平语义 (高=发送 / 低=接收)。
 * ============================================================ */
#define UART_OPENART_TX_PIN     C16     /* MAIN_OPENART1_UART_TX */
#define UART_OPENART_RX_PIN     C17     /* MAIN_OPENART1_UART_RX */

#endif /* _PIN_MAP_H_ */
