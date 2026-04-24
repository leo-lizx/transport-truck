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

/* 单轮 PID 调试参数（按需修改）
 * wheel_index: 0=LF 1=RF 2=LB 3=RB
 */
#define MAIN_PID_DEBUG_WHEEL_INDEX         (CHASSIS_WHEEL_LF)
#define MAIN_PID_DEBUG_TARGET_MPS          (8.0f)
#define MAIN_MENU_RENDER_DIV               (2U)

/*
 * 左前轮 PID 调试参数入口（按需修改）：
 * - 该组参数会在初始化阶段覆盖到运行时调参结构。
 * - 若置 0，则不在 main 内强制覆盖，继续沿用 Flash/菜单中的 PID。
 */
#define MAIN_PID_DEBUG_FORCE_LF_PID_FROM_MAIN   (0)
#define MAIN_PID_DEBUG_LF_KP                    (120.0f)
#define MAIN_PID_DEBUG_LF_KI                    (8.0f)
#define MAIN_PID_DEBUG_LF_KD                    (1.0f)

static void main_apply_left_front_pid_for_debug(void)
{
#if (1 == MAIN_PID_DEBUG_FORCE_LF_PID_FROM_MAIN)
    chassis_tune_params_t tune_params;

    /* 先读取完整参数，再仅覆盖左前轮 PID。 */
    chassis_ctrl_get_tune_params(&tune_params);
    tune_params.wheel_pid_kp[CHASSIS_WHEEL_LF] = MAIN_PID_DEBUG_LF_KP;
    tune_params.wheel_pid_ki[CHASSIS_WHEEL_LF] = MAIN_PID_DEBUG_LF_KI;
    tune_params.wheel_pid_kd[CHASSIS_WHEEL_LF] = MAIN_PID_DEBUG_LF_KD;
    chassis_ctrl_set_tune_params(&tune_params);
#endif
}

int main(void)
{
    uint8 menu_render_div = 0U;

    /* 1) 时钟/调试串口 */
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    /* 2) 菜单显示与按键 */
    ips200_set_dir(IPS200_CROSSWISE);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    key_init(10);

    /* 3) 底盘与菜单参数初始化 */
    chassis_ctrl_init();
    chassis_menu_init();

    /* 4) 按 main 顶部宏覆盖左前轮 PID 并启动单轮调试模式 */
    main_apply_left_front_pid_for_debug();
    chassis_ctrl_start_single_wheel_pid_debug((uint8)MAIN_PID_DEBUG_WHEEL_INDEX,
                                              MAIN_PID_DEBUG_TARGET_MPS);

    /* 5) 中断节拍：5ms 姿态、20ms 闭环、10ms 菜单按键 */
    pit_ms_init(PIT_CH0, 5);
    pit_ms_init(PIT_CH1, 20);
    pit_ms_init(PIT_CH2, 10);

    /* 6) 主循环：打印单轮 PID 调试信息 + 菜单渲染 */
    while (1)
    {
        chassis_pid_debug_task_5ms();

        menu_render_div++;
        if (menu_render_div >= MAIN_MENU_RENDER_DIV)
        {
            menu_render_div = 0U;
            chassis_menu_render_100ms();
        }

        system_delay_ms(5);
    }
}
