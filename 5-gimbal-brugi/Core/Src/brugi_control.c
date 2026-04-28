#include "brugi_control.h"

#define BRUGI_LOOP_HZ 500.0f
#define BRUGI_LOOP_DT (1.0f / BRUGI_LOOP_HZ)
#define BRUGI_STARTUP_IDLE_SEC 1.0f
#define BRUGI_STARTUP_UNLOCK_SEC 5.0f
#define BRUGI_PHASE_CENTER_DEG 180.0f
#define BRUGI_PHASE_LIMIT_DEG 85.0f
#define BRUGI_YAW_HOLD_PHASE_DEG 180.0f
static float clampf_local(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static MotorAxis motor_axis_from_index(uint8_t index)
{
    switch (index) {
    case 0:
        return AXIS_ROLL;
    case 1:
        return AXIS_PITCH;
    default:
        return AXIS_YAW;
    }
}

static void update_state_machine(BrugiControl *control)
{
    control->state_ticks++;

    switch (control->state) {
    case BRUGI_GIMBAL_IDLE:
        control->motors_enabled = 0;
        BrugiIMU_SetAccTimeConstant(&control->imu,
                                    control->config.acc_time_constant_startup,
                                    control->loop_dt_s);
        if (control->state_ticks >= (uint32_t)(BRUGI_STARTUP_IDLE_SEC * BRUGI_LOOP_HZ)) {
            control->state = BRUGI_GIMBAL_UNLOCKED;
            control->state_ticks = 0;
        }
        break;

    case BRUGI_GIMBAL_UNLOCKED:
        control->motors_enabled = 1;
        BrugiIMU_SetAccTimeConstant(&control->imu,
                                    control->config.acc_time_constant_startup,
                                    control->loop_dt_s);
        if (control->state_ticks >= (uint32_t)(BRUGI_STARTUP_UNLOCK_SEC * BRUGI_LOOP_HZ)) {
            control->state = BRUGI_GIMBAL_LOCKED;
            control->state_ticks = 0;
        }
        break;

    case BRUGI_GIMBAL_LOCKED:
        control->motors_enabled = 1;
        BrugiIMU_SetAccTimeConstant(&control->imu,
                                    control->config.acc_time_constant,
                                    control->loop_dt_s);
        break;

    case BRUGI_GIMBAL_ERROR:
    default:
        control->motors_enabled = 0;
        break;
    }
}

BrugiInitStatus BrugiControl_Init(BrugiControl *control,
                                  I2C_HandleTypeDef *hi2c,
                                  TIM_HandleTypeDef *htim2,
                                  TIM_HandleTypeDef *htim3,
                                  TIM_HandleTypeDef *htim4)
{
    BrugiConfig_LoadDefaults(&control->config);

    control->loop_dt_s = BRUGI_LOOP_DT;
    control->pitch_setpoint_deg = 0.0f;
    control->roll_setpoint_deg = 0.0f;
    control->pitch_phase_center_deg = BRUGI_PHASE_CENTER_DEG;
    control->roll_phase_center_deg = BRUGI_PHASE_CENTER_DEG;
    control->pitch_phase_cmd_deg = BRUGI_PHASE_CENTER_DEG;
    control->roll_phase_cmd_deg = BRUGI_PHASE_CENTER_DEG;
    control->state_ticks = 0;
    control->motors_enabled = 0;
    control->state = BRUGI_GIMBAL_IDLE;
    control->init_status = BRUGI_INIT_ERR_CONTROL;

    Motor_InitAll(htim2, htim3, htim4);
    Motor_Coast(AXIS_YAW);

    if (!BrugiIMU_Init(&control->imu, hi2c, &control->config)) {
        control->state = BRUGI_GIMBAL_ERROR;
        control->init_status = BRUGI_INIT_ERR_MPU_NOT_FOUND;
        return control->init_status;
    }

    BrugiIMU_SetAccTimeConstant(&control->imu,
                                control->config.acc_time_constant,
                                control->loop_dt_s);

    if (control->config.gyro_calibrate_on_startup) {
        BrugiIMU_Calibrate(&control->imu, 500);
    }

    PID_Init(&control->pitch_pid,
             control->config.pitch_kp,
             control->config.pitch_ki,
             control->config.pitch_kd,
             30.0f,
             BRUGI_PHASE_LIMIT_DEG);
    PID_Init(&control->roll_pid,
             control->config.roll_kp,
             control->config.roll_ki,
             control->config.roll_kd,
             30.0f,
             BRUGI_PHASE_LIMIT_DEG);

    control->init_status = BRUGI_INIT_OK;
    return control->init_status;
}

void BrugiControl_Update(BrugiControl *control)
{
    float pitch_out;
    float roll_out;
    MotorAxis pitch_axis;
    MotorAxis roll_axis;

    if (control->state == BRUGI_GIMBAL_ERROR) {
        Motor_CoastAll();
        return;
    }

    update_state_machine(control);
    BrugiIMU_Update(&control->imu, control->loop_dt_s);

    if (!control->motors_enabled) {
        Motor_Coast(motor_axis_from_index(control->config.motor_number_pitch));
        Motor_Coast(motor_axis_from_index(control->config.motor_number_roll));
        Motor_Coast(AXIS_YAW);
        return;
    }

    pitch_out = PID_Compute(&control->pitch_pid,
                            control->pitch_setpoint_deg,
                            control->imu.pitch_deg,
                            control->loop_dt_s);
    roll_out = PID_Compute(&control->roll_pid,
                           control->roll_setpoint_deg,
                           control->imu.roll_deg,
                           control->loop_dt_s);

    control->pitch_phase_cmd_deg = clampf_local(
        control->pitch_phase_center_deg + control->config.dir_motor_pitch * pitch_out,
        control->pitch_phase_center_deg - BRUGI_PHASE_LIMIT_DEG,
        control->pitch_phase_center_deg + BRUGI_PHASE_LIMIT_DEG);
    control->roll_phase_cmd_deg = clampf_local(
        control->roll_phase_center_deg + control->config.dir_motor_roll * roll_out,
        control->roll_phase_center_deg - BRUGI_PHASE_LIMIT_DEG,
        control->roll_phase_center_deg + BRUGI_PHASE_LIMIT_DEG);

    pitch_axis = motor_axis_from_index(control->config.motor_number_pitch);
    roll_axis = motor_axis_from_index(control->config.motor_number_roll);

    Motor_SetPhase(pitch_axis,
                   control->pitch_phase_cmd_deg,
                   control->config.max_pwm_motor_pitch);
    Motor_SetPhase(roll_axis,
                   control->roll_phase_cmd_deg,
                   control->config.max_pwm_motor_roll);

    Motor_SetPhase(AXIS_YAW,
                   BRUGI_YAW_HOLD_PHASE_DEG,
                   control->config.max_pwm_motor_yaw_hold);
}
