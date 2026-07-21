#ifndef CHASSIS_VELOCITY_HANDOFF_H
#define CHASSIS_VELOCITY_HANDOFF_H

/* Distance-based bumpless transfer from MPC velocity to terminal PID velocity. */
static inline float chassis_velocity_handoff_blend(float remaining_dist_m,
                                                   float blend_start_m,
                                                   float blend_end_m,
                                                   float mpc_velocity_mps,
                                                   float pid_velocity_mps)
{
    float mpc_weight;

    if (remaining_dist_m >= blend_start_m) {
        return mpc_velocity_mps;
    }
    if ((remaining_dist_m <= blend_end_m) ||
        (blend_start_m <= blend_end_m)) {
        return pid_velocity_mps;
    }

    mpc_weight = (remaining_dist_m - blend_end_m) /
                 (blend_start_m - blend_end_m);
    return mpc_weight * mpc_velocity_mps +
           (1.0f - mpc_weight) * pid_velocity_mps;
}

#endif /* CHASSIS_VELOCITY_HANDOFF_H */
