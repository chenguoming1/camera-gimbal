#ifndef GIMBAL_MOTOR_H
#define GIMBAL_MOTOR_H
#include <stdint.h>
int Gimbal_MotorInit(void);
int Gimbal_MotorHealthy(void);
int Gimbal_MotorArm(void);
int Gimbal_MotorApply(const float phase[3], const uint16_t power[3]);
void Gimbal_MotorStop(void);
#endif
