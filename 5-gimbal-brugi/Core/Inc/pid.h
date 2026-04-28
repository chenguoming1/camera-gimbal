/* =============================================================
 * pid.h
 * Discrete PID controller — one instance per gimbal axis
 * =============================================================*/
#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    /* Gains */
    float kp, ki, kd;

    /* Anti-windup: clamp integrator and output */
    float i_limit;     /* max absolute integral contribution */
    float out_limit;   /* max absolute output value         */

    /* State */
    float integral;
    float prev_error;
    uint8_t first_run;
} PID_Handle;

/* API */
void  PID_Init(PID_Handle *pid, float kp, float ki, float kd,
               float i_limit, float out_limit);
float PID_Compute(PID_Handle *pid, float setpoint,
                  float measured, float dt);
void  PID_Reset(PID_Handle *pid);

#endif /* PID_H */
