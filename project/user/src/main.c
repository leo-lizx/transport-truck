#include "zf_common_headfile.h"
#include "chassis_ctrl.h"
#include "chassis_menu.h"
#include "app_main_config.h"
#include "app_main_modes.h"
#include "app_main_runtime.h"
#include <stdio.h>

int main(void)
{
    uint8 menu_render_div = 0U;

    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    App_MainRuntime_SetupStdout();

    uart_write_string(UART_1, "HW:OK\r\n");
    printf("BOOT mode=%d\n", (int)MAIN_RUN_MODE);

    App_MainModes_InitCommunication();

    ips200_set_dir(IPS200_CROSSWISE);
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    key_init(10);

    chassis_ctrl_init();
    chassis_menu_init();
    chassis_ctrl_set_pose(MAIN_POS_GRID_TO_M_X(MAIN_POS_NAV_START_X_GRID),
                          MAIN_POS_GRID_TO_M_Y(MAIN_POS_NAV_START_Y_GRID),
                          0.0f);
    App_MainModes_AfterChassisInit();

    pit_ms_init(PIT_CH0, 5);
    pit_ms_init(PIT_CH1, 20);
    pit_ms_init(PIT_CH2, 10);

    while (1)
    {
        App_MainRuntime_PrintLoopEnteredOnce();
        (void)App_MainRuntime_WaitForTick();
        App_MainRuntime_PrintTickHeartbeat();

        App_MainModes_Task5ms();

        menu_render_div++;
        if ((App_MainModes_ShouldRenderMenu() != 0U) &&
            (menu_render_div >= MAIN_MENU_RENDER_DIV))
        {
            menu_render_div = 0U;
            chassis_menu_render_100ms();
        }

        fflush(stdout);
    }
}

