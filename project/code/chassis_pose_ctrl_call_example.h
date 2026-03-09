#ifndef CHASSIS_POSE_CTRL_CALL_EXAMPLE_H
#define CHASSIS_POSE_CTRL_CALL_EXAMPLE_H

/*===========================================================================
 * [chassis_pose_ctrl_call_example.h]
 *
 *   main.c  -> app_control_pipeline_init()
 *   isr.c   -> app_control_pipeline_on_pit_5ms / on_pit_20ms
 *   app     -> app_control_pipeline_move_to_grid / is_arrived
 *===========================================================================*/

#include "chassis_ctrl.h"

void app_control_pipeline_init(void);
void app_control_pipeline_on_pit_5ms(void);
void app_control_pipeline_on_pit_20ms(void);

void app_control_pipeline_move_to_grid(uint8 target_x_grid, uint8 target_y_grid);
uint8 app_control_pipeline_is_arrived(void);

void app_control_pipeline_correct_pose(float x_m, float y_m, float yaw_deg);
void app_control_pipeline_set_gyro_bias(float gyro_z_bias_dps);

#endif /* CHASSIS_POSE_CTRL_CALL_EXAMPLE_H */