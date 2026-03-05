/* ===========================================================
 * motor.h
 * DRV8313 Motor Driver — Static FOC Position Hold
 * Motor 0: IN1=PA7 (TIM3_CH2), IN2=PB0 (TIM3_CH3), IN3=PB1 (TIM3_CH4)
 * Target: STM32F103RCT6
 * ===========================================================*/

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

/* ---- User config ---------------------------------------- */
#define PWM_ARR             999    /* Must match TIM3 ARR value  */
#define HOLD_STRENGTH       40     /* 0-100%, how hard it holds  */
/* --------------------------------------------------------- */

/* Public API */
void Motor_Init(TIM_HandleTypeDef *htim);

/* Hold rotor at a fixed electrical angle.
 * angle_deg : 0..359  electrical degrees
 * strength  : 0..100  percent of max torque */
void Motor_HoldPosition(float angle_deg, uint8_t strength);

/* Release all phases (motor free to spin) */
void Motor_Coast(void);

/* Slowly step through angles so you can see the motor move,
 * then freezes at the final position. Good for first test. */
void Motor_SweepTest(void);

#endif /* MOTOR_H */

