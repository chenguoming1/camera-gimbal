/* ===========================================================
 * motor.h
 * DRV8313 Open-Loop 6-Step Commutation Driver
 * Motor 0: IN1=PA7 (TIM3_CH2), IN2=PB0 (TIM3_CH3), IN3=PB1 (TIM3_CH4)
 * Target: STM32F103RCT6
 * ===========================================================*/

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

/* ---- User config ---------------------------------------- */
#define PWM_DUTY_PERCENT    40    /* 0-100%, start low ~30-40% */
#define STEP_DELAY_MS       5     /* ms per commutation step   */
#define PWM_ARR             999   /* Must match TIM3 ARR value */
/* --------------------------------------------------------- */

/* Public API */
void Motor_Init(TIM_HandleTypeDef *htim);
void Motor_Loop(void);   /* Does not return — call from main() */

#endif /* MOTOR_H */
