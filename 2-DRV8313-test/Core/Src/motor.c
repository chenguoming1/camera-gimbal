/* ===========================================================
 * motor.c
 * DRV8313 Motor Driver — Static FOC Position Hold
 *
 * HOW POSITION HOLD WORKS:
 *   A 3-phase BLDC motor has 3 coils 120° apart. If you inject
 *   a sinusoidal current pattern into all 3 phases at a fixed
 *   electrical angle, the rotor's permanent magnet locks onto
 *   the resulting magnetic field and holds that position.
 *
 *   Phase A = sin(angle)
 *   Phase B = sin(angle + 120°)
 *   Phase C = sin(angle + 240°)
 *
 *   Since the DRV8313 can only source (not sink) on each output,
 *   we shift and scale the sine waves to the range 0..ARR:
 *
 *   CCR = (sin(angle) + 1.0) / 2.0 * ARR * strength/100
 *
 * ===========================================================*/

#include "main.h"
#include "motor.h"
#include <math.h>

/* ---- Private defines ------------------------------------ */
#define DEG2RAD(x)   ((x) * 3.14159265f / 180.0f)

/* ---- Private variables ---------------------------------- */
static TIM_HandleTypeDef *_htim = NULL;

/* ---- Private functions ---------------------------------- */

/* Apply raw compare values directly to the 3 TIM3 channels */
static void SetPhases(uint32_t ccr_a, uint32_t ccr_b, uint32_t ccr_c)
{
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_2, ccr_a); /* PA7 - IN1 */
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_3, ccr_b); /* PB0 - IN2 */
    __HAL_TIM_SET_COMPARE(_htim, TIM_CHANNEL_4, ccr_c); /* PB1 - IN3 */
}

/* Convert a sine value (-1..1) to a PWM compare register value.
 * Shifts and scales into 0..ARR range, then applies strength. */
static uint32_t SineToCCR(float sine_val, uint8_t strength)
{
    float scaled = (sine_val + 1.0f) / 2.0f;          /* 0.0 .. 1.0  */
    scaled *= (float)(PWM_ARR + 1);                    /* 0 .. ARR+1  */
    scaled *= (float)strength / 100.0f;                /* apply gain  */
    if (scaled > PWM_ARR) scaled = PWM_ARR;
    return (uint32_t)scaled;
}

/* ---- Public functions ----------------------------------- */

void Motor_Init(TIM_HandleTypeDef *htim)
{
    _htim = htim;

    HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_2); /* PA7 - IN1 */
    HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_3); /* PB0 - IN2 */
    HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_4); /* PB1 - IN3 */

    SetPhases(0, 0, 0);
    HAL_Delay(100);
}

void Motor_HoldPosition(float angle_deg, uint8_t strength)
{
    /* Compute the 3-phase sinusoidal voltages at this angle */
    float a = sinf(DEG2RAD(angle_deg));
    float b = sinf(DEG2RAD(angle_deg + 120.0f));
    float c = sinf(DEG2RAD(angle_deg + 240.0f));

    SetPhases(SineToCCR(a, strength),
              SineToCCR(b, strength),
              SineToCCR(c, strength));
}

void Motor_Coast(void)
{
    /* All phases off — rotor spins freely */
    SetPhases(0, 0, 0);
}

void Motor_SweepTest(void)
{
    /* --- Phase 1: Slowly rotate 2 full electrical cycles ---
     * This confirms the motor moves and all phases are wired correctly.
     * Watch the shaft — it should rotate smoothly.              */
    for (int i = 0; i < 720; i++)
    {
        Motor_HoldPosition((float)i, HOLD_STRENGTH);
        HAL_Delay(10);   /* 10ms per degree = ~3.6 sec per revolution */
    }

    /* --- Phase 2: Freeze at 0° and hold ---
     * Motor should now resist any attempt to rotate it by hand.
     * If it does, your motor and DRV8313 are working correctly. */
    while (1)
    {
        Motor_HoldPosition(0.0f, HOLD_STRENGTH);
        HAL_Delay(1);
    }
}

