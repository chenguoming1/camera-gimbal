/* =============================================================
 * mpu6050.h
 * MPU6050 I2C driver — single device, board IMU
 *
 * Board IMU : I2C1 (PB6=SCL, PB7=SDA), address 0x69 (AD0 HIGH)
 * =============================================================*/
#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

/* 7-bit I2C address — AD0 pulled HIGH on board IMU */
#define MPU_ADDR   0x69

/* Register map (subset) */
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CFG     0x1B
#define MPU_REG_ACCEL_CFG    0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_GYRO_XOUT_H  0x43
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_WHO_AM_I     0x75

typedef struct {
    float roll;  /* Degrees */
    float pitch; /* Degrees */
    uint32_t last_time;
} MPU_Orientation;

/* Gyro full-scale range */
typedef enum {
    MPU_GYRO_250DPS  = 0x00,
    MPU_GYRO_500DPS  = 0x08,
    MPU_GYRO_1000DPS = 0x10,
    MPU_GYRO_2000DPS = 0x18,
} MPU_GyroRange;

/* Accel full-scale range */
typedef enum {
    MPU_ACCEL_2G  = 0x00,
    MPU_ACCEL_4G  = 0x08,
    MPU_ACCEL_8G  = 0x10,
    MPU_ACCEL_16G = 0x18,
} MPU_AccelRange;

/* Raw sensor counts */
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} MPU_Raw;

/* Scaled sensor data */
typedef struct {
    float ax, ay, az;   /* m/s²  */
    float gx, gy, gz;   /* deg/s */
} MPU_Data;

/* Device handle */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    float  gyro_lsb;            /* deg/s per LSB  */
    float  accel_lsb;           /* g per LSB      */
    /* Calibration offsets (raw counts) */
    int16_t ax_off, ay_off, az_off;
    int16_t gx_off, gy_off, gz_off;
} MPU_Handle;

/* API */
uint8_t MPU_Init(MPU_Handle *mpu, I2C_HandleTypeDef *hi2c,
                 MPU_GyroRange gyro_range, MPU_AccelRange accel_range);
void    MPU_ReadRaw(MPU_Handle *mpu, MPU_Raw *raw);
void    MPU_ReadScaled(MPU_Handle *mpu, MPU_Data *data);
void    MPU_Calibrate(MPU_Handle *mpu, uint16_t samples);

#endif /* MPU6050_H */
