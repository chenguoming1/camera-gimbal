#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

#define MPU6050_ADDR         0x68
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_TEMP_OUT_H   0x41

typedef struct {
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
    int16_t Temp;
} MPU6050_Data;

void MPU6050_Init(I2C_HandleTypeDef *hi2c);
uint8_t MPU6050_ReadByte(I2C_HandleTypeDef *hi2c, uint8_t reg);
void MPU6050_ReadData(I2C_HandleTypeDef *hi2c, MPU6050_Data *data);

#endif
