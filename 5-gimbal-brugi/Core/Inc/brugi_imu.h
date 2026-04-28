#ifndef BRUGI_IMU_H
#define BRUGI_IMU_H

#include "brugi_config.h"
#include "mpu6050.h"

typedef struct {
    MPU_Handle mpu;
    const BrugiConfig *config;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float accel_alpha;
} BrugiImu;

uint8_t BrugiIMU_Init(BrugiImu *imu,
                      I2C_HandleTypeDef *hi2c,
                      const BrugiConfig *config);
void BrugiIMU_Calibrate(BrugiImu *imu, uint16_t samples);
void BrugiIMU_SetAccTimeConstant(BrugiImu *imu, float seconds, float dt);
void BrugiIMU_Update(BrugiImu *imu, float dt);

#endif
