#ifndef BRUGI_CONTROL_H
#define BRUGI_CONTROL_H

#include "brugi_config.h"
#include "brugi_imu.h"
#include "motor.h"
#include "pid.h"

typedef enum {
    BRUGI_GIMBAL_IDLE = 0,
    BRUGI_GIMBAL_UNLOCKED,
    BRUGI_GIMBAL_LOCKED,
    BRUGI_GIMBAL_ERROR
} BrugiGimbalState;

typedef enum {
    BRUGI_INIT_OK = 0,
    BRUGI_INIT_ERR_MPU_NOT_FOUND = 1,
    BRUGI_INIT_ERR_CONTROL = 2
} BrugiInitStatus;

typedef struct {
    BrugiConfig config;
    BrugiImu imu;

    PID_Handle pitch_pid;
    PID_Handle roll_pid;

    float pitch_setpoint_deg;
    float roll_setpoint_deg;

    float pitch_phase_center_deg;
    float roll_phase_center_deg;
    float pitch_phase_cmd_deg;
    float roll_phase_cmd_deg;

    float loop_dt_s;
    uint32_t state_ticks;
    uint8_t motors_enabled;
    BrugiGimbalState state;
    BrugiInitStatus init_status;
} BrugiControl;

BrugiInitStatus BrugiControl_Init(BrugiControl *control,
                                  I2C_HandleTypeDef *hi2c,
                                  TIM_HandleTypeDef *htim2,
                                  TIM_HandleTypeDef *htim3,
                                  TIM_HandleTypeDef *htim4);
void BrugiControl_Update(BrugiControl *control);

#endif
