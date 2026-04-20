/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 RT1064DVL6A 开源库的一部分
*
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "chassis_ctrl.h"
#include "chassis_pid.h"
#include "chassis_menu.h"
#include "app_game_logic.h"

/*
 * main 运行模式：
 * 1 = 航向保持测试模式（默认）：进入后保持目标航向角
 * 0 = 游戏状态机模式
 */
#define MAIN_YAW_HOLD_TEST_MODE   (1)

/* 单轮 PID 调试模式：
 * 1 = 启用（每次只调一个轮子 PID）
 * 0 = 关闭（保持原有航向/游戏逻辑）
 */
#define MAIN_SINGLE_WHEEL_PID_DEBUG_MODE   (1)

/* 单轮 PID 调试参数（按需修改） */
#define MAIN_PID_DEBUG_WHEEL_INDEX         (CHASSIS_WHEEL_RF)
#define MAIN_PID_DEBUG_TARGET_MPS          (0.40f)

/* 航向保持测试目标角（单位：度，可按需修改） */
#define MAIN_HOLD_YAW_TARGET_DEG  (0.0f)

/*==========================================================================
 *  推箱子智能车完整调用流程 (第二十一届全国智能车竞赛 AI视觉组)
 *
 *  ┌─────────────────────────────────────────────────────────────────┐
 *  │ 系统初始化                                                       │
 *  │   clock_init  →  debug_init  →  uart_init (OpenART 通信)       │
 *  │   chassis_ctrl_init (IMU零偏/电机/编码器/PID)                    │
 *  │   pit_ms_init (5ms 姿态 + 20ms 控制 + 10ms 菜单)                │
 *  └──────────────────────────┬──────────────────────────────────────┘
 *                             ▼
 *  ┌─────────── while(1) 主循环 ──────────────────────────────────────┐
 *  │                                                                  │
 *  │  Game_Logic_Task_Run()  ← 游戏状态机 (非阻塞, 每轮调一次)       │
 *  │                                                                  │
 *  │  状态机流转:                                                      │
 *  │    PENDING_SCOUT ──→ 去最近箱子, 看前摄像头分阶段               │
 *  │         │ class_id==0               │ class_id>0                 │
 *  │         ▼                           ▼                            │
 *  │    STAGE_1_BASIC_EXEC         OBSERVE_ALL                       │
 *  │    (任意推箱, 推完→DONE)       (看遍所有箱子)                    │
 *  │                                  │ 无炸弹        │ 有炸弹        │
 *  │                                  ▼               ▼               │
 *  │                          STAGE_2_CLASS    STAGE_3_STRATEGY       │
 *  │                          (分类推箱→DONE)  (尝试分类推箱)         │
 *  │                                            │ 无解                │
 *  │                                            ▼                     │
 *  │                                     找不可达目标                  │
 *  │                                     Find_Bomb_Wall()             │
 *  │                                     Solve_Push_Bomb()            │
 *  │                                            │                     │
 *  │                                            ▼                     │
 *  │                                     STAGE_3_BOMB_PUSH           │
 *  │                                     (推炸弹到墙→爆炸)            │
 *  │                                            │                     │
 *  │                                            ▼                     │
 *  │                                     回到 STAGE_3 重试            │
 *  │                                     (循环直到推完→DONE)          │
 *  │                                                                  │
 *  └──────────────────────────────────────────────────────────────────┘
 *
 *  中断层 (isr.c):
 *    PIT_CH0  5ms  → chassis_ctrl_task_5ms()   航向角积分与姿态更新
 *    PIT_CH1  20ms → chassis_ctrl_task_20ms()  底盘闭环控制
 *    PIT_CH2  10ms → chassis_menu_task_10ms()  菜单按键扫描（渲染在主循环）
 *    UART_1         → OpenART 串口帧解析, 更新 g_game_map
 *==========================================================================*/
int main(void)
{
    uint8 menu_render_div = 0U;

    clock_init(SYSTEM_CLOCK_600M);  // 不可删除
    debug_init();                   // 调试端口初始化

    // ------------------------------------------------------------------
    // 1. 通信外设初始化
    // ------------------------------------------------------------------
    // 串口1: 与 OpenART 视觉模块通信 (波特率需与 OpenART 一致)
#if (0 == MAIN_YAW_HOLD_TEST_MODE)
    uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);
    uart_rx_interrupt(UART_1, 1);
#endif

    // ------------------------------------------------------------------
    // 2. IPS200 屏幕 + 按键初始化 (调参菜单)
    // ------------------------------------------------------------------
    ips200_set_dir(IPS200_CROSSWISE);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    key_init(10);

    // ------------------------------------------------------------------
    // 3. 底盘控制子系统初始化 + 调参菜单
    //    包括: IMU 零偏标定、电机 PWM、编码器、PID 参数
    //    注意: 调用此函数前请确保车模静止放置在平面上
    // ------------------------------------------------------------------
    chassis_ctrl_init();
    chassis_menu_init();
    // 发车位修正：第一列，距最底边界约 1m（折算到网格后为 G(1,7)）。
    chassis_ctrl_set_pose(chassis_grid_x_to_m(CHASSIS_START_GRID_X),
                          chassis_grid_y_to_m(CHASSIS_START_GRID_Y),
                          0.0f);

#if (1 == MAIN_YAW_HOLD_TEST_MODE)
#if (1 == MAIN_SINGLE_WHEEL_PID_DEBUG_MODE)
    /* 单轮 PID 调试模式：
     * - 仅 MAIN_PID_DEBUG_WHEEL_INDEX 参与闭环跟踪
     * - 其他轮子目标固定为 0
     * - menu 参数仍可实时生效，长按 K4 可走现有 flash 写入保存
     */
    chassis_ctrl_start_single_wheel_pid_debug((uint8)MAIN_PID_DEBUG_WHEEL_INDEX,
                                              MAIN_PID_DEBUG_TARGET_MPS);
#else
    // 航向保持测试模式：仅设置一次目标航向，不运行推箱状态机。
    // 持续闭环由 PIT_CH1 的 chassis_ctrl_task_20ms() 执行。
    chassis_ctrl_hold_yaw(MAIN_HOLD_YAW_TARGET_DEG);
#endif
#else
    // 游戏模式：仅设置初始航向基准。
    chassis_ctrl_hold_yaw(0.0f);
#endif

    // ------------------------------------------------------------------
    // 4. PIT 定时中断初始化
    //    CH0: 5ms 姿态采样  CH1: 20ms 底盘闭环  CH2: 10ms 菜单扫描（渲染在主循环）
    // ------------------------------------------------------------------
    pit_ms_init(PIT_CH0, 5);
    pit_ms_init(PIT_CH1, 20);
    pit_ms_init(PIT_CH2, 10);

    // ------------------------------------------------------------------
    // 5. 主循环: 游戏状态机
    //    Game_Logic_Task_Run 为非阻塞函数，内部维护推箱子状态机。
    // ------------------------------------------------------------------
    // 静态调试阶段可先不运行状态机，改为手动下发网格目标点。
    // app_chassis_ctrl_move_to_grid(3, 5);

    while (1)
    {
    #if (1 == MAIN_SINGLE_WHEEL_PID_DEBUG_MODE)
        /* 单轮 PID 调试输出：每 100ms 仅打印“目标值 实际值”两列数字。 */
        chassis_pid_debug_task_5ms();
    #else
        // 航向闭环调试任务：100ms 打印一次目标角/当前角/误差/角速度指令。
        chassis_ctrl_attitude_debug_task_5ms();
    #endif

    #if (1 == MAIN_YAW_HOLD_TEST_MODE)
        /*
         * 航向保持测试模式：
         * 目标角已在初始化阶段设置，20ms 中断中持续执行闭环。
         * 这里不重复调用 hold_yaw，避免反复清空积分项影响闭环效果。
         */
    #else
        Game_Logic_Task_Run();                         // 运行推箱子游戏状态机 (非阻塞)
    #endif

        /* 菜单渲染放到主循环，避免在 PIT 中断内刷屏造成控制节拍抖动。 */
        menu_render_div++;
        if (menu_render_div >= 2U)
        {
            menu_render_div = 0U;
            chassis_menu_render_100ms();
        }

        system_delay_ms(5);                               // 主循环节拍
    }
}
