#ifndef GIMBAL_APP_H
#define GIMBAL_APP_H
#include <stdint.h>
#include "gimbal_control.h"
typedef enum { GIMBAL_WAIT_IMU, GIMBAL_CALIBRATING, GIMBAL_DISARMED, GIMBAL_RAMP, GIMBAL_ACTIVE, GIMBAL_FAULT } Gimbal_State;
typedef enum { GIMBAL_OK, GIMBAL_STOP, GIMBAL_BUTTON, GIMBAL_IMU_ERROR, GIMBAL_STALE, GIMBAL_TIMING, GIMBAL_TIMER, GIMBAL_TILT, GIMBAL_RX_OVERFLOW, GIMBAL_CPU_FAULT } Gimbal_Fault;
typedef struct {
  Gimbal_State state;
  Gimbal_Fault fault;
  uint8_t who_am_i, address, orientation_ok, direction_ok;
  uint32_t samples, errors, last_sample_ms, calibration_samples, control_us, max_control_us;
  uint32_t last_i2c_error;
  int16_t accel_raw[3], gyro_raw[3];
  float accel_g[3], gyro_dps[3], bias_dps[3];
  /* All indexed arrays use roll,pitch,yaw; their motor ports are MOT2,MOT1,MOT0. */
  float angle[3], target[3], phase[3];
  int axes[3], direction[3], enabled[3];
  uint16_t power[3], compare[3][3];
  uint8_t request_stop; /* Debugger stop request; never resumes automatically. */
  uint32_t command_count, command_errors;
  char reply[128];
} Gimbal_Status;
extern volatile Gimbal_Status gimbal;
void Gimbal_Init(void);
void Gimbal_Task(void);
void Gimbal_Tick(void);
void Gimbal_EmergencyStop(void);
void Gimbal_Receive(const uint8_t *data, uint32_t length);
void Gimbal_Report(void); /* Stable, non-I2C breakpoint for SWD diagnostics. */
#endif
