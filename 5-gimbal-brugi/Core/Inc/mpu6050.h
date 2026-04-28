/* =============================================================
 * mpu6050.h
 * MPU6050 driver — supports one or more devices on the same I2C bus
 *
 * BruGi port uses a single MPU6050 on I2C2 (PB10/PB11), but the driver keeps
 * support for either AD0 state so it can auto-detect 0x68 or 0x69.
 * =============================================================*/
#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

/* I2C 7-bit addresses */
#define MPU_ADDR_BOARD   0x69   /* board IMU,  AD0 = HIGH */
#define MPU_ADDR_CAM     0x68   /* camera IMU, AD0 = LOW  */

/* Register map (subset) */
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CFG     0x1B
#define MPU_REG_ACCEL_CFG    0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_TEMP_OUT_H   0x41
#define MPU_REG_GYRO_XOUT_H  0x43
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_WHO_AM_I     0x75

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

/* Raw sensor data */
typedef struct {
    int16_t ax, ay, az;   /* raw accelerometer counts */
    int16_t gx, gy, gz;   /* raw gyroscope counts     */
    int16_t temp;         /* raw temperature count    */
} MPU_Raw;

/* Scaled sensor data (SI units) */
typedef struct {
    float ax, ay, az;   /* m/s² */
    float gx, gy, gz;   /* deg/s */
} MPU_Data;

/* Per-device handle */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t            addr;     /* 7-bit address, shifted <<1 inside driver */
    float              gyro_lsb; /* deg/s per LSB */
    float              accel_lsb;/* g per LSB     */
    /* Calibration offsets (in raw counts) */
    int16_t            ax_off, ay_off, az_off;
    int16_t            gx_off, gy_off, gz_off;
} MPU_Handle;

/* API */
uint8_t MPU_Init(MPU_Handle *mpu, I2C_HandleTypeDef *hi2c, uint8_t addr,
                 MPU_GyroRange gyro_range, MPU_AccelRange accel_range);
void    MPU_ReadRaw(MPU_Handle *mpu, MPU_Raw *raw);
void    MPU_ReadScaled(MPU_Handle *mpu, MPU_Data *data);
void    MPU_Calibrate(MPU_Handle *mpu, uint16_t samples);

#endif /* MPU6050_H */
