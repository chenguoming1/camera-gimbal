/* =============================================================
 * imu.c
 * Complementary filter angle estimation — dual MPU6050
 * =============================================================*/
#include "imu.h"
#include <math.h>

#define RAD2DEG  (180.0f / 3.14159265f)

/* ---------- private: accel-only angle estimate ------------ */

static void accel_angles(MPU_Data *d, float *roll, float *pitch)
{
    /* roll  = atan2(ay, az) */
    *roll  = atan2f(d->ay, d->az) * RAD2DEG;
    /* pitch = atan2(-ax, sqrt(ay^2 + az^2)) */
    *pitch = atan2f(-d->ax, sqrtf(d->ay * d->ay + d->az * d->az)) * RAD2DEG;
}

/* ---------- private: update one IMU + filter -------------- */

static void update_one(MPU_Handle *mpu, IMU_Angles *angles)
{
    MPU_Data d;
    MPU_ReadScaled(mpu, &d);

    float accel_roll, accel_pitch;
    accel_angles(&d, &accel_roll, &accel_pitch);

    /* Complementary filter */
    angles->roll  = CF_ALPHA * (angles->roll  + d.gx * IMU_DT)
                  + (1.0f - CF_ALPHA) * accel_roll;
    angles->pitch = CF_ALPHA * (angles->pitch + d.gy * IMU_DT)
                  + (1.0f - CF_ALPHA) * accel_pitch;

    /* Yaw is gyro-only (no absolute reference without magnetometer) */
    angles->yaw  += d.gz * IMU_DT;

    /* Wrap yaw to ±180° */
    if (angles->yaw >  180.0f) angles->yaw -= 360.0f;
    if (angles->yaw < -180.0f) angles->yaw += 360.0f;
}

/* ---------- public API ------------------------------------ */

uint8_t IMU_Init(IMU_State *imu,
                 I2C_HandleTypeDef *hi2c)
{
    uint8_t ok = 1;

    /* Board IMU: I2C1, address 0x69 (AD0 HIGH) */
    ok &= MPU_Init(&imu->board, hi2c, MPU_ADDR_BOARD,
                   MPU_GYRO_500DPS, MPU_ACCEL_4G);

    /* Camera IMU: I2C1, address 0x68 (AD0 LOW) */
    ok &= MPU_Init(&imu->cam, hi2c, MPU_ADDR_CAM,
                   MPU_GYRO_500DPS, MPU_ACCEL_4G);

    /* Zero angle state */
    imu->board_angle.roll  = 0.0f;
    imu->board_angle.pitch = 0.0f;
    imu->board_angle.yaw   = 0.0f;
    imu->cam_angle.roll    = 0.0f;
    imu->cam_angle.pitch   = 0.0f;
    imu->cam_angle.yaw     = 0.0f;
    imu->error.roll        = 0.0f;
    imu->error.pitch       = 0.0f;
    imu->error.yaw         = 0.0f;

    return ok;
}

void IMU_Calibrate(IMU_State *imu)
{
    MPU_Calibrate(&imu->board, 500);
    MPU_Calibrate(&imu->cam,   500);
}

void IMU_Update(IMU_State *imu)
{
    update_one(&imu->board, &imu->board_angle);
    update_one(&imu->cam,   &imu->cam_angle);

    /* Error = camera angle relative to world frame
     * The gimbal task is to drive this error to zero */
    imu->error.roll  = imu->cam_angle.roll  - imu->board_angle.roll;
    imu->error.pitch = imu->cam_angle.pitch - imu->board_angle.pitch;
    imu->error.yaw   = imu->cam_angle.yaw   - imu->board_angle.yaw;
}
