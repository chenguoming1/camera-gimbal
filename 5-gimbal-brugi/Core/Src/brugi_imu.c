#include "brugi_imu.h"

#include <math.h>

#define RAD2DEG (180.0f / 3.14159265f)

static void map_sensor_axes(const BrugiConfig *config,
                            float raw_x, float raw_y, float raw_z,
                            float *roll, float *pitch, float *yaw)
{
    float gyro_roll = raw_x;
    float gyro_pitch = -raw_y;
    float gyro_yaw = raw_z;

    if (config->axis_reverse_z) {
        gyro_pitch = -gyro_pitch;
        gyro_yaw = -gyro_yaw;
    }

    if (config->axis_swap_xy) {
        float tmp = gyro_roll;
        gyro_roll = gyro_pitch;
        gyro_pitch = tmp;
        gyro_yaw = -gyro_yaw;
    }

    *roll = gyro_roll;
    *pitch = gyro_pitch;
    *yaw = gyro_yaw;
}

static void map_acc_axes(const BrugiConfig *config,
                         float raw_x, float raw_y, float raw_z,
                         float *roll_acc, float *pitch_acc, float *yaw_acc)
{
    float acc_roll = raw_y;
    float acc_pitch = raw_x;
    float acc_yaw = raw_z;

    if (config->axis_reverse_z) {
        acc_roll = -acc_roll;
        acc_yaw = -acc_yaw;
    }

    if (config->axis_swap_xy) {
        float tmp = acc_roll;
        acc_roll = acc_pitch;
        acc_pitch = tmp;
    }

    *roll_acc = acc_roll;
    *pitch_acc = acc_pitch;
    *yaw_acc = acc_yaw;
}

uint8_t BrugiIMU_Init(BrugiImu *imu,
                      I2C_HandleTypeDef *hi2c,
                      const BrugiConfig *config)
{
    uint8_t ok = MPU_Init(&imu->mpu, hi2c, MPU_ADDR_BOARD,
                          MPU_GYRO_500DPS, MPU_ACCEL_4G);
    if (!ok) {
        ok = MPU_Init(&imu->mpu, hi2c, MPU_ADDR_CAM,
                      MPU_GYRO_500DPS, MPU_ACCEL_4G);
    }
    if (!ok) {
        return 0;
    }

    imu->config = config;
    imu->roll_deg = 0.0f;
    imu->pitch_deg = 0.0f;
    imu->yaw_deg = 0.0f;
    imu->accel_alpha = 0.95f;
    return 1;
}

void BrugiIMU_Calibrate(BrugiImu *imu, uint16_t samples)
{
    MPU_Calibrate(&imu->mpu, samples);
    imu->roll_deg = 0.0f;
    imu->pitch_deg = 0.0f;
    imu->yaw_deg = 0.0f;
}

void BrugiIMU_SetAccTimeConstant(BrugiImu *imu, float seconds, float dt)
{
    float tau = seconds;
    if (tau < dt) {
        tau = dt;
    }

    imu->accel_alpha = tau / (tau + dt);
    if (imu->accel_alpha < 0.0f) {
        imu->accel_alpha = 0.0f;
    }
    if (imu->accel_alpha > 1.0f) {
        imu->accel_alpha = 1.0f;
    }
}

void BrugiIMU_Update(BrugiImu *imu, float dt)
{
    MPU_Data data;
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float accel_roll;
    float accel_pitch;
    float alpha = imu->accel_alpha;
    float roll_est = imu->roll_deg - imu->config->angle_offset_roll_deg;
    float pitch_est = imu->pitch_deg - imu->config->angle_offset_pitch_deg;

    MPU_ReadScaled(&imu->mpu, &data);

    map_sensor_axes(imu->config, data.gx, data.gy, data.gz, &gx, &gy, &gz);
    map_acc_axes(imu->config, data.ax, data.ay, data.az, &ax, &ay, &az);

    if (imu->config->enable_gyro) {
        roll_est += gx * dt;
        pitch_est += gy * dt;
        imu->yaw_deg += gz * dt;
    }

    accel_roll = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
    accel_pitch = atan2f(ay, az) * RAD2DEG;

    if (imu->config->enable_acc) {
        roll_est = alpha * roll_est + (1.0f - alpha) * accel_roll;
        pitch_est = alpha * pitch_est + (1.0f - alpha) * accel_pitch;
    }

    imu->roll_deg = roll_est + imu->config->angle_offset_roll_deg;
    imu->pitch_deg = pitch_est + imu->config->angle_offset_pitch_deg;

    if (imu->yaw_deg > 180.0f) {
        imu->yaw_deg -= 360.0f;
    } else if (imu->yaw_deg < -180.0f) {
        imu->yaw_deg += 360.0f;
    }
}
