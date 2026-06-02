/*
 * app_main_modes.h
 *
 * 主循环运行模式分发接口。运行模式由 main.c 在启动入口显式配置，
 * 避免把模式选择隐藏在预处理宏里。
 */

#ifndef APP_MAIN_MODES_H_
#define APP_MAIN_MODES_H_

#include "chassis_config.h"
#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 运行模式枚举。
 * 由 main.c 通过 App_MainModes_Config() 在启动阶段写入，主循环周期内只读。
 * 不同模式下 App_MainModes_Task5ms() 分发到不同业务入口，互斥执行，不可在运行中动态切换。
 * 调试模式（YAW_HOLD / SINGLE_WHEEL / POINT_NAV 等）不启动视觉链路，节省串口资源。
 */
typedef enum
{
    /* 完整比赛流程：视觉地图、三阶段推箱状态机、死局/掉线/越界处理。 */
    APP_RUN_MODE_GAME = 0,

    /* 航向保持调试：只验证 IMU yaw 环和底盘原地稳定性。 */
    APP_RUN_MODE_YAW_HOLD,

    /* 单轮速度闭环调试：用于标定轮 PID、编码器方向和起步补偿。 */
    APP_RUN_MODE_SINGLE_WHEEL,

    /* 点到点导航调试：按内置航点数组验证定位、直线性和到点判定。 */
    APP_RUN_MODE_POINT_NAV,

    /* 推箱求解器离线自测：只打印算法结果，不下发底盘运动。 */
    APP_RUN_MODE_SOKO_SELFTEST,

    /* OpenART1 串口链路测试：验证 MAP/心跳帧解析和链路统计。 */
    APP_RUN_MODE_OPENART1_TEST,

    /* 静态地图自动执行：用一个固定/锁定地图求解并下发完整航点数组。 */
    APP_RUN_MODE_STATIC_MAP_DRIVE
} app_main_run_mode_e;

typedef enum
{
    /* 静态地图来自 OpenART1 MAP 帧，适合联调视觉链路。 */
    APP_STATIC_MAP_SOURCE_OPENART1 = 0,

    /* 静态地图来自固件内置 ASCII 地图，适合纯底盘和算法闭环验证。 */
    APP_STATIC_MAP_SOURCE_MANUAL
} app_static_map_source_e;

typedef struct
{
    /* 单位: 枚举；默认: APP_RUN_MODE_STATIC_MAP_DRIVE；建议范围: app_main_run_mode_e。
     * 影响: 决定主循环 5ms tick 分发到哪个业务模式。 */
    app_main_run_mode_e run_mode;

    /* 单位: 枚举；默认: APP_STATIC_MAP_SOURCE_MANUAL；建议范围: app_static_map_source_e。
     * 影响: 决定静态地图模式使用内置地图还是等待 OpenART1 MAP 帧。 */
    app_static_map_source_e static_map_source;

    /* 单位: 轮子索引；默认: CHASSIS_WHEEL_LF；建议范围: [0, CHASSIS_WHEEL_COUNT)。
     * 影响: SINGLE_WHEEL 模式下选择被调试的轮子。 */
    uint8 single_wheel_index;

    /* 单位: 布尔量；默认: 1；建议范围: 0/1。
     * 影响: 1=进入单轮调试前临时覆盖该轮 PID，0=使用当前活动调参。 */
    uint8 single_wheel_force_pid;

    /* 单位: m/s；默认: 3.3；建议范围: 由 chassis_ctrl 内部调试速度限幅决定。
     * 影响: SINGLE_WHEEL 模式下该轮的目标线速度。 */
    float single_wheel_target_mps;

    /* 单位: 无量纲；默认: 40；建议范围: [0,400]。
     * 影响: SINGLE_WHEEL 临时覆盖的轮速 PID 比例项，增大跟随更快但更易抖。 */
    float single_wheel_pid_kp;

    /* 单位: 无量纲；默认: 23；建议范围: [0,80]。
     * 影响: SINGLE_WHEEL 临时覆盖的轮速 PID 积分项，增大可减小稳态误差但更易振荡。 */
    float single_wheel_pid_ki;

    /* 单位: 无量纲；默认: 0；建议范围: [0,40]。
     * 影响: SINGLE_WHEEL 临时覆盖的轮速 PID 微分项，增大阻尼但可能放大噪声。 */
    float single_wheel_pid_kd;

    /* 单位: deg；默认: 0；建议范围: [-180,180]。
     * 影响: YAW_HOLD 模式下底盘保持的目标航向角。 */
    float hold_yaw_target_deg;
} app_main_options_t;

/*
 * 在调用其他 App_MainModes_* 初始化接口前应用运行时模式配置。
 * options 为空或枚举值非法时使用安全默认值。
 */
void App_MainModes_Config(const app_main_options_t *options);

/*
 * 初始化当前模式需要的通信外设。
 * 调用时机: main.c 中底盘初始化之前调用一次。
 */
void App_MainModes_InitCommunication(void);

/*
 * 底盘初始化完成后启动当前模式的控制状态。
 */
void App_MainModes_AfterChassisInit(void);

/*
 * 将一次 5ms 主循环 tick 分发给当前模式。
 */
void App_MainModes_Task5ms(void);

/*
 * 返回 1 表示允许普通底盘菜单刷新 IPS 屏幕。
 */
uint8 App_MainModes_ShouldRenderMenu(void);

/*
 * 返回 1 表示当前为静态地图自动执行模式。
 * 该接口只读取模式枚举，可被轻量中断钩子查询。
 */
uint8 App_MainModes_IsStaticMapDrive(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_MODES_H_ */
