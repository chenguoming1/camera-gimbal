/* =============================================================
 * pid.c
 * Discrete PID controller
 * =============================================================*/
#include "pid.h"

/* clamp helper */
static float clampf(float v, float limit)
{
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

void PID_Init(PID_Handle *pid, float kp, float ki, float kd,
              float i_limit, float out_limit)
{
    pid->kp        = kp;
    pid->ki        = ki;
    pid->kd        = kd;
    pid->i_limit   = i_limit;
    pid->out_limit = out_limit;
    PID_Reset(pid);
}

float PID_Compute(PID_Handle *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;

    /* Proportional */
    float p = pid->kp * error;

    /* Integral with anti-windup clamp */
    pid->integral += error * dt;
    pid->integral  = clampf(pid->integral, pid->i_limit);
    float i = pid->ki * pid->integral;

    /* Derivative (on measurement to avoid derivative kick on setpoint change) */
    float d = 0.0f;
    if (!pid->first_run) {
        d = -pid->kd * (measured - pid->prev_error) / dt;
    }
    pid->first_run  = 0;
    pid->prev_error = measured;

    float output = clampf(p + i + d, pid->out_limit);
    return output;
}

void PID_Reset(PID_Handle *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->first_run  = 1;
}
