/* =============================================================
 * gimbal.c
 * Top-level stabilization coordinator
 *
 * Call Gimbal_Update() from the TIM1 update interrupt at 1 kHz.
 * =============================================================*/
#include "gimbal.h"

uint8_t Gimbal_Init(Gimbal_State *g,
                    I2C_HandleTypeDef *hi2c,   /* both IMUs on I2C1 */
                    TIM_HandleTypeDef *htim2,
                    TIM_HandleTypeDef *htim3,
                    TIM_HandleTypeDef *htim4)
{
    /* --- Motors -------------------------------------------- */
    Motor_InitAll(htim2, htim3, htim4);

    /* --- IMU ----------------------------------------------- */
    uint8_t ok = IMU_Init(&g->imu, hi2c);
    if (!ok) return 0;

    /* Calibrate with board flat and stationary (~1 second) */
    IMU_Calibrate(&g->imu);

    /* --- PIDs ---------------------------------------------- */
    /* args: kp, ki, kd, integrator_limit, output_limit(deg elec) */
    PID_Init(&g->pid_roll,  ROLL_KP,  ROLL_KI,  ROLL_KD,  30.0f, 90.0f);
    PID_Init(&g->pid_pitch, PITCH_KP, PITCH_KI, PITCH_KD, 30.0f, 90.0f);
    PID_Init(&g->pid_yaw,   YAW_KP,   YAW_KI,   YAW_KD,   30.0f, 90.0f);

    /* --- State --------------------------------------------- */
    g->target_roll  = 0.0f;
    g->target_pitch = 0.0f;
    g->target_yaw   = 0.0f;
    g->elec_roll    = 0.0f;
    g->elec_pitch   = 0.0f;
    g->elec_yaw     = 0.0f;
    g->running      = 1;

    return 1;
}

void Gimbal_Update(Gimbal_State *g)
{
    if (!g->running) return;

    /* 1. Read both IMUs and compute error angles */
    IMU_Update(&g->imu);

    /* 2. PID: error → torque demand (in electrical degrees/tick) */
    float dt = IMU_DT;

    float cmd_roll  = PID_Compute(&g->pid_roll,
                                   g->target_roll,
                                   g->imu.error.roll,  dt);
    float cmd_pitch = PID_Compute(&g->pid_pitch,
                                   g->target_pitch,
                                   g->imu.error.pitch, dt);
    float cmd_yaw   = PID_Compute(&g->pid_yaw,
                                   g->target_yaw,
                                   g->imu.error.yaw,   dt);

    /* 3. Accumulate electrical angle
     * The PID output is an angular velocity command (deg/s).
     * Multiply by dt to get incremental electrical angle step. */
    g->elec_roll  += cmd_roll  * dt;
    g->elec_pitch += cmd_pitch * dt;
    g->elec_yaw   += cmd_yaw   * dt;

    /* Wrap electrical angles to 0..360 */
    if (g->elec_roll  < 0.0f) g->elec_roll  += 360.0f;
    if (g->elec_roll  >= 360.0f) g->elec_roll  -= 360.0f;
    if (g->elec_pitch < 0.0f) g->elec_pitch += 360.0f;
    if (g->elec_pitch >= 360.0f) g->elec_pitch -= 360.0f;
    if (g->elec_yaw   < 0.0f) g->elec_yaw   += 360.0f;
    if (g->elec_yaw   >= 360.0f) g->elec_yaw   -= 360.0f;

    /* 4. Drive motors */
    Motor_SetPhase(AXIS_ROLL,  g->elec_roll,  GIMBAL_STRENGTH);
    Motor_SetPhase(AXIS_PITCH, g->elec_pitch, GIMBAL_STRENGTH);
    Motor_SetPhase(AXIS_YAW,   g->elec_yaw,   GIMBAL_STRENGTH);
}

void Gimbal_SetTarget(Gimbal_State *g, float roll, float pitch, float yaw)
{
    g->target_roll  = roll;
    g->target_pitch = pitch;
    g->target_yaw   = yaw;
}

void Gimbal_Stop(Gimbal_State *g)
{
    g->running = 0;
    Motor_CoastAll();
    PID_Reset(&g->pid_roll);
    PID_Reset(&g->pid_pitch);
    PID_Reset(&g->pid_yaw);
}
