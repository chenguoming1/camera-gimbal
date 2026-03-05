#include "mpu6050.h"

void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2];
    data[0] = MPU6050_PWR_MGMT_1;
    data[1] = 0;
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR << 1, data, 2, 100);
}

uint8_t MPU6050_ReadByte(I2C_HandleTypeDef *hi2c, uint8_t reg) {
    uint8_t value;
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR << 1, &reg, 1, 100);
    HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR << 1, &value, 1, 100);
    return value;
}

void MPU6050_ReadData(I2C_HandleTypeDef *hi2c, MPU6050_Data *data) {
    uint8_t buffer[14];
    buffer[0] = MPU6050_ACCEL_XOUT_H;
    HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR << 1, buffer, 1, 100);
    HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR << 1, buffer, 14, 100);

    data->AccX = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->AccY = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->AccZ = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->Temp  = (int16_t)((buffer[6] << 8) | buffer[7]);
    data->GyroX = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->GyroY = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->GyroZ = (int16_t)((buffer[12] << 8) | buffer[13]);
}
