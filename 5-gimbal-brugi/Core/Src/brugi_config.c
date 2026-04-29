#include "brugi_config.h"

void BrugiConfig_LoadDefaults(BrugiConfig *config)
{
    config->pitch_kp = 3.0f;
    config->pitch_ki = 0.20f;
    config->pitch_kd = 0.05f;
    config->roll_kp = 3.0f;
    config->roll_ki = 0.20f;
    config->roll_kd = 0.05f;

    config->acc_time_constant = 7.0f;
    config->acc_time_constant_startup = 2.0f;

    config->angle_offset_pitch_deg = 0.0f;
    config->angle_offset_roll_deg = 0.0f;

    config->dir_motor_pitch = 1;
    config->dir_motor_roll = -1;
    config->motor_number_pitch = 1;
    config->motor_number_roll = 0;
    config->max_pwm_motor_pitch = 65;
    config->max_pwm_motor_roll = 70;
    config->max_pwm_motor_yaw_hold = 45;

    config->enable_gyro = 1;
    config->enable_acc = 1;
    config->axis_reverse_z = 1;
    config->axis_swap_xy = 0;
    config->gyro_calibrate_on_startup = 1;
}
