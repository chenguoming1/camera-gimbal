/* =============================================================
 * gimbal.h
 * Top-level gimbal coordinator
 * =============================================================*/
#ifndef GIMBAL_H
#define GIMBAL_H

#include "imu.h"
#include "motor.h"
#include "pid.h"

/* Motor hold strength: 0..100 % */
#define GIMBAL_STRENGTH     45

/* Default PID gains — tune these for your motors */
#define ROLL_KP   3.0f
#define ROLL_KI   0.02f
#define ROLL_KD   0.05f

#define PITCH_KP  3.0f
#define PITCH_KI  0.02f
#define PITCH_KD  0.05f

#define YAW_KP    2.0f
#define YAW_KI    0.01f
#define YAW_KD    0.03f

typedef struct {
    IMU_State  imu;
    PID_Handle pid_roll;
    PID_Handle pid_pitch;
    PID_Handle pid_yaw;

    /* Target angles (degrees) — 0 = hold horizon */
    float target_roll;
    float target_pitch;
    float target_yaw;

    /* Motor electrical angle accumulators */
    float elec_roll;
    float elec_pitch;
    float elec_yaw;

    uint8_t  running;
} Gimbal_State;

/* API */
uint8_t Gimbal_Init(Gimbal_State *g,
                    I2C_HandleTypeDef *hi2c,   /* both IMUs on I2C1 */
                    TIM_HandleTypeDef *htim2,
                    TIM_HandleTypeDef *htim3,
                    TIM_HandleTypeDef *htim4);

/* Call from TIM1 interrupt at 1 kHz */
void Gimbal_Update(Gimbal_State *g);

void Gimbal_SetTarget(Gimbal_State *g, float roll, float pitch, float yaw);
void Gimbal_Stop(Gimbal_State *g);

#endif /* GIMBAL_H */
