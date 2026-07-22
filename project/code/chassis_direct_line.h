#ifndef CHASSIS_DIRECT_LINE_H
#define CHASSIS_DIRECT_LINE_H

#include <math.h>
#include <stdint.h>

/** 固定起终点直线的单位切向量与法向量；只保存几何，不持有控制状态。 */
typedef struct
{
    float start_x_m;
    float start_y_m;
    float ux;
    float uy;
    float nx;
    float ny;
} chassis_direct_line_t;

/** 由起终点生成固定直线；零长度路径返回 0，避免单位向量除零。 */
static inline uint8_t chassis_direct_line_init(
    chassis_direct_line_t *line,
    float start_x_m,
    float start_y_m,
    float target_x_m,
    float target_y_m)
{
    float dx;
    float dy;
    float length;

    if (line == 0) return 0U;

    dx = target_x_m - start_x_m;
    dy = target_y_m - start_y_m;
    length = sqrtf(dx * dx + dy * dy);
    if (length <= 1e-6f) return 0U;

    line->start_x_m = start_x_m;
    line->start_y_m = start_y_m;
    line->ux = dx / length;
    line->uy = dy / length;
    line->nx = -line->uy;
    line->ny = line->ux;
    return 1U;
}

/** 将当前位置误差分解为沿线剩余量和指向固定直线的横向纠偏量。 */
static inline void chassis_direct_line_error(
    const chassis_direct_line_t *line,
    float pose_x_m,
    float pose_y_m,
    float target_x_m,
    float target_y_m,
    float *along_error_m,
    float *cross_error_m)
{
    float target_dx = target_x_m - pose_x_m;
    float target_dy = target_y_m - pose_y_m;
    float from_start_x = pose_x_m - line->start_x_m;
    float from_start_y = pose_y_m - line->start_y_m;

    *along_error_m = target_dx * line->ux + target_dy * line->uy;
    *cross_error_m = -(from_start_x * line->nx + from_start_y * line->ny);
}

/** 将全局速度投影到固定直线的切向和法向。 */
static inline void chassis_direct_line_project_velocity(
    const chassis_direct_line_t *line,
    float vx_global_mps,
    float vy_global_mps,
    float *along_mps,
    float *cross_mps)
{
    *along_mps = vx_global_mps * line->ux + vy_global_mps * line->uy;
    *cross_mps = vx_global_mps * line->nx + vy_global_mps * line->ny;
}

/** 按直线方向的 X/Y 分量加权已有轴参数，纯 X/Y 时退化为原轴参数。 */
static inline float chassis_direct_line_weighted_param(
    const chassis_direct_line_t *line,
    float x_value,
    float y_value)
{
    float wx = fabsf(line->ux);
    float wy = fabsf(line->uy);
    float sum = wx + wy;

    return (sum > 1e-6f) ? ((wx * x_value + wy * y_value) / sum)
                         : x_value;
}

#endif /* CHASSIS_DIRECT_LINE_H */
