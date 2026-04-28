#ifndef BRUGI_CONFIG_H
#define BRUGI_CONFIG_H

#include <stdint.h>

typedef struct {
    float pitch_kp;
    float pitch_ki;
    float pitch_kd;
    float roll_kp;
    float roll_ki;
    float roll_kd;

    float acc_time_constant;
    float acc_time_constant_startup;

    float angle_offset_pitch_deg;
    float angle_offset_roll_deg;

    int8_t dir_motor_pitch;
    int8_t dir_motor_roll;
    uint8_t motor_number_pitch;
    uint8_t motor_number_roll;
    uint8_t max_pwm_motor_pitch;
    uint8_t max_pwm_motor_roll;
    uint8_t max_pwm_motor_yaw_hold;

    uint8_t enable_gyro;
    uint8_t enable_acc;
    uint8_t axis_reverse_z;
    uint8_t axis_swap_xy;
    uint8_t gyro_calibrate_on_startup;
} BrugiConfig;

void BrugiConfig_LoadDefaults(BrugiConfig *config);

#endif
