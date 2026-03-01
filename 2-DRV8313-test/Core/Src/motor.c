/* ===========================================================
 * motor.c
 * DRV8313 Open-Loop 6-Step Commutation Driver
 * Motor 0: IN1=PA7 (TIM3_CH2), IN2=PB0 (TIM3_CH3), IN3=PB1 (TIM3_CH4)
 * Target: STM32F103RCT6
 * ===========================================================*/

#include "main.h"
#include "motor.h"

/* ---- Private types -------------------------------------- */
typedef struct
{
    uint32_t a;   /* Phase A (PA7 - TIM3_CH2) */
    uint32_t b;   /* Phase B (PB0 - TIM3_CH3) */
    uint32_t c;   /* Phase C (PB1 - TIM3_CH4) */
} PhaseState;

/* ---- Private variables ---------------------------------- */
static TIM_HandleTypeDef *_htim = NULL;

/* ---- 6-Step commutation table ---------------------------
 *
 *  Step |  A(PA7) | B(PB0) | C(PB1) | Current path
 *  -----+---------+--------+--------+-------------
 *   0   |   PWM   |   0    |   0    | A → B
 *   1   |   PWM   |   0    |   0    | A → C
 *   2   |    0    |  PWM   |   0    | B → C
 *   3   |    0    |  PWM   |   0    | B → A
 *   4   |    0    |   0    |  PWM   | C → A
 *   5   |    0    |   0    |  PWM   | C → B
 *
 *  INx = PWM  → high side switching, current flows out
 *  INx = 0    → low side on, current flows in (return path)
 * --------------------------------------------------------- */
static const PhaseState COMMUTATION_TABLE[6] =
{
    /* A      B      C  */
    {1,      0,     0},   /* step 0: A+ B- */
    {1,      0,     0},   /* step 1: A+ C- */
    {0,      1,     0},   /* step 2: B+ C- */
    {0,      1,     0},   /* step 3: B+ A- */
    {0,      0,     1},   /* step 4: C+ A- */
    {0,      0,     1},   /* step 5: C+ B- */
};

/* ---- Private functions ---------------------------------- */

/* Convert duty % to timer compare value */
static uint32_t DutyToCCR(uint32_t percent)
{
    return (percent * (PWM_ARR + 1)) / 100;
}

/* Apply phase voltages to TIM3 channels */
static void SetPhases(uint32_t dutyA, uint32_t dutyB, uint32_t dutyC)
{
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, dutyA); /* PA7 - IN1 */
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_3, dutyB); /* PB0 - IN2 */
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_4, dutyC); /* PB1 - IN3 */
}

/* Execute one electrical step from the commutation table */
static void CommutateStep(uint8_t step, uint32_t duty)
{
    const PhaseState *s = &COMMUTATION_TABLE[step];
    SetPhases(s->a * duty, s->b * duty, s->c * duty);
}

/* Run one full electrical cycle (6 steps = 1 mechanical revolution
 * for a 1 pole-pair motor, or a fraction for multi-pole motors)    */
static void Motor_RunCycle(uint32_t duty, uint32_t step_delay_ms)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        CommutateStep(i, duty);
        HAL_Delay(step_delay_ms);
    }
}

/* ---- Public functions ----------------------------------- */

void Motor_Init(TIM_HandleTypeDef *htim)
{
    _htim = htim;

    /* Start PWM output on all 3 channels */
    HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_2); /* PA7 - IN1 */
    HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_3); /* PB0 - IN2 */
    HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_4); /* PB1 - IN3 */

    /* All phases off initially */
    SetPhases(0, 0, 0);
    HAL_Delay(100);
}

void Motor_Loop(void)
{
    uint32_t duty = DutyToCCR(PWM_DUTY_PERCENT);

    while (1)
    {
        Motor_RunCycle(duty, STEP_DELAY_MS);
    }
}
