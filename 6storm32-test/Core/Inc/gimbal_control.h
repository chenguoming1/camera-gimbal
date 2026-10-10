#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H
#include <stdint.h>
/* Right-handed camera body: X=lens forward, Y=right, Z=down. Angles in degrees. */
typedef struct { float q[4], angle[3], rate[3]; } Gimbal_Attitude;
typedef struct { float kp, ki, kd, integral, phase; } Gimbal_PID;
float Gimbal_Clamp(float x, float lo, float hi);
float Gimbal_Wrap(float degrees);
int Gimbal_ValidAxes(const int axes[3]);
void Gimbal_Map(const float raw[3], const int axes[3], float body[3]);
void Gimbal_AttitudeInit(Gimbal_Attitude *s, const float accel[3]);
int Gimbal_AttitudeUpdate(Gimbal_Attitude *s, const float accel[3], const float gyro[3], float dt);
float Gimbal_PIDStep(Gimbal_PID *p, float error, float rate, float dt, int direction);
void Gimbal_PWM(float phase, unsigned strength, uint16_t compare[3]);
#endif
