/* =============================================================
 * motor.h
 * 3-axis BLDC gimbal motor driver — DRV8313, sinusoidal FOC
 *
 * Mot0 (Roll)  : TIM3_CH2=PA7, TIM3_CH3=PB0,  TIM3_CH4=PB1
 * Mot1 (Pitch) : TIM3_CH1=PA6, TIM2_CH4=PA3,  TIM2_CH3=PA2
 * Mot2 (Yaw)   : TIM4_CH4=PB9, TIM2_CH2=PA1,  TIM4_CH3=PB8
 * =============================================================*/
#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

#define PWM_ARR          999    /* Must match TIM ARR */
#define MOTOR_AXES       3

typedef enum {
    AXIS_ROLL  = 0,
    AXIS_PITCH = 1,
    AXIS_YAW   = 2,
} MotorAxis;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           ch_a;
    uint32_t           ch_b;
    uint32_t           ch_c;
    float              elec_angle;  /* current commanded electrical angle */
    uint8_t            enabled;
} Motor_Handle;

/* API */
void Motor_InitAll(TIM_HandleTypeDef *htim2,
                   TIM_HandleTypeDef *htim3,
                   TIM_HandleTypeDef *htim4);
void Motor_SetPhase(MotorAxis axis, float angle_deg, float strength);
void Motor_Coast(MotorAxis axis);
void Motor_CoastAll(void);

#endif /* MOTOR_H */
