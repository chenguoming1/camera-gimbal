/* =============================================================
 * motor.c
 * 3-axis sinusoidal motor driver — DRV8313
 *
 * Mot0 (Roll)  : TIM3  CH2=PA7, CH3=PB0, CH4=PB1
 * Mot1 (Pitch) : TIM3  CH1=PA6, TIM2 CH4=PA3, TIM2 CH3=PA2
 * Mot2 (Yaw)   : TIM4  CH4=PB9, TIM2 CH2=PA1, TIM4 CH3=PB8
 * =============================================================*/
#include "motor.h"
#include <math.h>

#define DEG2RAD(x)  ((x) * 3.14159265f / 180.0f)

static Motor_Handle _motors[MOTOR_AXES];

/* TIM2 handle shared by the split-timer axes (Pitch and Yaw) */
static TIM_HandleTypeDef *_htim2_ref = NULL;

/* ---- private: sine wave → PWM compare value -------------- */
/* angle_rad : electrical angle in radians
 * norm      : normalised strength, 0.0 .. 1.0               */
static uint32_t sine_to_ccr(float angle_rad, float norm)
{
    float s = (sinf(angle_rad) + 1.0f) * 0.5f;   /* 0..1 */
    s *= norm;                                     /* 0..norm */
    uint32_t ccr = (uint32_t)(s * (float)PWM_ARR);
    if (ccr > PWM_ARR) ccr = PWM_ARR;
    return ccr;
}

/* ---- public API ------------------------------------------ */

void Motor_InitAll(TIM_HandleTypeDef *htim2,
                   TIM_HandleTypeDef *htim3,
                   TIM_HandleTypeDef *htim4)
{
    /* Mot0 Roll: all three channels on TIM3 */
    _motors[AXIS_ROLL].htim  = htim3;
    _motors[AXIS_ROLL].ch_a  = TIM_CHANNEL_2;  /* PA7 */
    _motors[AXIS_ROLL].ch_b  = TIM_CHANNEL_3;  /* PB0 */
    _motors[AXIS_ROLL].ch_c  = TIM_CHANNEL_4;  /* PB1 */

    /* Mot1 Pitch: TIM3_CH1 + TIM2_CH4 + TIM2_CH3
     * Note: split across two timers — ch_a is on TIM3, b/c on TIM2.
     * We handle this by writing directly with two separate timer handles. */
    _motors[AXIS_PITCH].htim  = htim3;          /* ch_a handle */
    _motors[AXIS_PITCH].ch_a  = TIM_CHANNEL_1;  /* PA6 */
    _motors[AXIS_PITCH].ch_b  = TIM_CHANNEL_4;  /* PA3 (TIM2) */
    _motors[AXIS_PITCH].ch_c  = TIM_CHANNEL_3;  /* PA2 (TIM2) */

    /* Mot2 Yaw: TIM4_CH4 + TIM2_CH2 + TIM4_CH3 */
    _motors[AXIS_YAW].htim  = htim4;
    _motors[AXIS_YAW].ch_a  = TIM_CHANNEL_4;   /* PB9 */
    _motors[AXIS_YAW].ch_b  = TIM_CHANNEL_2;   /* PA1 (TIM2) */
    _motors[AXIS_YAW].ch_c  = TIM_CHANNEL_3;   /* PB8 */

    /* Start all PWM channels */
    HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(htim4, TIM_CHANNEL_4);

    for (int i = 0; i < MOTOR_AXES; i++) {
        _motors[i].elec_angle = 0.0f;
        _motors[i].enabled    = 1;
    }

    /* Store htim2 pointer for the split-timer axes (Pitch and Yaw) */
    _htim2_ref = htim2;

    HAL_Delay(50);
}

void Motor_SetPhase(MotorAxis axis, float angle_deg, float strength)
{
    float a = DEG2RAD(angle_deg);
    float b = DEG2RAD(angle_deg + 120.0f);
    float c = DEG2RAD(angle_deg + 240.0f);
    float s = strength / 100.0f;   /* normalise to 0..1 */

    if (axis == AXIS_ROLL) {
        /* All three channels on TIM3 */
        __HAL_TIM_SET_COMPARE(_motors[AXIS_ROLL].htim,
                              TIM_CHANNEL_2, sine_to_ccr(a, s));
        __HAL_TIM_SET_COMPARE(_motors[AXIS_ROLL].htim,
                              TIM_CHANNEL_3, sine_to_ccr(b, s));
        __HAL_TIM_SET_COMPARE(_motors[AXIS_ROLL].htim,
                              TIM_CHANNEL_4, sine_to_ccr(c, s));

    } else if (axis == AXIS_PITCH) {
        /* CH_A on TIM3, CH_B and CH_C on TIM2 */
        __HAL_TIM_SET_COMPARE(_motors[AXIS_PITCH].htim,
                              TIM_CHANNEL_1, sine_to_ccr(a, s));
        if (_htim2_ref) {
            __HAL_TIM_SET_COMPARE(_htim2_ref,
                                  TIM_CHANNEL_4, sine_to_ccr(b, s));
            __HAL_TIM_SET_COMPARE(_htim2_ref,
                                  TIM_CHANNEL_3, sine_to_ccr(c, s));
        }

    } else { /* AXIS_YAW */
        /* CH_A on TIM4, CH_B on TIM2, CH_C on TIM4 */
        __HAL_TIM_SET_COMPARE(_motors[AXIS_YAW].htim,
                              TIM_CHANNEL_4, sine_to_ccr(a, s));
        if (_htim2_ref) {
            __HAL_TIM_SET_COMPARE(_htim2_ref,
                                  TIM_CHANNEL_2, sine_to_ccr(b, s));
        }
        __HAL_TIM_SET_COMPARE(_motors[AXIS_YAW].htim,
                              TIM_CHANNEL_3, sine_to_ccr(c, s));
    }
}

void Motor_Coast(MotorAxis axis)
{
    Motor_SetPhase(axis, 0.0f, 0.0f);
    _motors[axis].enabled = 0;
}

void Motor_CoastAll(void)
{
    for (int i = 0; i < MOTOR_AXES; i++) {
        _motors[i].enabled = 1;  /* re-enable to allow SetPhase to write */
        Motor_SetPhase((MotorAxis)i, 0.0f, 0.0f);
        _motors[i].enabled = 0;
    }
}
