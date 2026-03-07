/* =============================================================
 * mpu6050.c
 * MPU6050 I2C driver — single device, board IMU (addr 0x69)
 * =============================================================*/
#include "mpu6050.h"
#include <math.h>

/* ---------- private helpers -------------------------------- */

static void mpu_write(MPU_Handle *mpu, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    HAL_I2C_Master_Transmit(mpu->hi2c, MPU_ADDR << 1, buf, 2, 10);
}

static uint8_t mpu_read_byte(MPU_Handle *mpu, uint8_t reg)
{
    uint8_t val = 0;
    HAL_I2C_Master_Transmit(mpu->hi2c, MPU_ADDR << 1, &reg, 1, 10);
    HAL_I2C_Master_Receive (mpu->hi2c, MPU_ADDR << 1, &val, 1, 10);
    return val;
}

static void mpu_read_burst(MPU_Handle *mpu, uint8_t reg,
                           uint8_t *buf, uint16_t len)
{
    HAL_I2C_Master_Transmit(mpu->hi2c, MPU_ADDR << 1, &reg, 1, 10);
    HAL_I2C_Master_Receive (mpu->hi2c, MPU_ADDR << 1, buf, len, 20);
}

/* ---------- public API ------------------------------------- */

uint8_t MPU_Init(MPU_Handle *mpu, I2C_HandleTypeDef *hi2c,
                 MPU_GyroRange gyro_range, MPU_AccelRange accel_range)
{
    mpu->hi2c   = hi2c;
    mpu->ax_off = mpu->ay_off = mpu->az_off = 0;
    mpu->gx_off = mpu->gy_off = mpu->gz_off = 0;

    HAL_Delay(100);

    /* Verify device identity */
    uint8_t who = mpu_read_byte(mpu, MPU_REG_WHO_AM_I);
    if (who != 0x68 && who != 0x69 && who != 0x70) return 0;

    /* Wake up, use PLL with X gyro as clock source */
    mpu_write(mpu, MPU_REG_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    /* Sample rate divider: 0 → 1 kHz */
    mpu_write(mpu, MPU_REG_SMPLRT_DIV, 0x00);

    /* DLPF: ~94 Hz bandwidth */
    mpu_write(mpu, MPU_REG_CONFIG, 0x02);

    /* Gyro range */
    mpu_write(mpu, MPU_REG_GYRO_CFG, (uint8_t)gyro_range);
    switch (gyro_range) {
        case MPU_GYRO_250DPS:  mpu->gyro_lsb = 1.0f / 131.0f;  break;
        case MPU_GYRO_500DPS:  mpu->gyro_lsb = 1.0f / 65.5f;   break;
        case MPU_GYRO_1000DPS: mpu->gyro_lsb = 1.0f / 32.8f;   break;
        default:               mpu->gyro_lsb = 1.0f / 16.4f;   break;
    }

    /* Accel range */
    mpu_write(mpu, MPU_REG_ACCEL_CFG, (uint8_t)accel_range);
    switch (accel_range) {
        case MPU_ACCEL_2G:  mpu->accel_lsb = 1.0f / 16384.0f; break;
        case MPU_ACCEL_4G:  mpu->accel_lsb = 1.0f / 8192.0f;  break;
        case MPU_ACCEL_8G:  mpu->accel_lsb = 1.0f / 4096.0f;  break;
        default:            mpu->accel_lsb = 1.0f / 2048.0f;  break;
    }

    return 1;
}

void MPU_ReadRaw(MPU_Handle *mpu, MPU_Raw *raw)
{
    uint8_t buf[14];
    mpu_read_burst(mpu, MPU_REG_ACCEL_XOUT_H, buf, 14);

    raw->ax = (int16_t)((buf[0]  << 8) | buf[1]);
    raw->ay = (int16_t)((buf[2]  << 8) | buf[3]);
    raw->az = (int16_t)((buf[4]  << 8) | buf[5]);
    /* buf[6..7] = temp, skip */
    raw->gx = (int16_t)((buf[8]  << 8) | buf[9]);
    raw->gy = (int16_t)((buf[10] << 8) | buf[11]);
    raw->gz = (int16_t)((buf[12] << 8) | buf[13]);

    /* Apply calibration offsets */
    raw->ax -= mpu->ax_off;
    raw->ay -= mpu->ay_off;
    raw->az -= mpu->az_off;
    raw->gx -= mpu->gx_off;
    raw->gy -= mpu->gy_off;
    raw->gz -= mpu->gz_off;
}

void MPU_ReadScaled(MPU_Handle *mpu, MPU_Data *data)
{
    MPU_Raw raw;
    MPU_ReadRaw(mpu, &raw);
    data->ax = raw.ax * mpu->accel_lsb * 9.80665f;
    data->ay = raw.ay * mpu->accel_lsb * 9.80665f;
    data->az = raw.az * mpu->accel_lsb * 9.80665f;
    data->gx = raw.gx * mpu->gyro_lsb;
    data->gy = raw.gy * mpu->gyro_lsb;
    data->gz = raw.gz * mpu->gyro_lsb;
}

//void MPU_Calibrate(MPU_Handle *mpu, uint16_t samples)
//{
//    int32_t ax_s = 0, ay_s = 0, az_s = 0;
//    int32_t gx_s = 0, gy_s = 0, gz_s = 0;
//
//    mpu->ax_off = mpu->ay_off = mpu->az_off = 0;
//    mpu->gx_off = mpu->gy_off = mpu->gz_off = 0;
//
//    for (uint16_t i = 0; i < samples; i++) {
//        MPU_Raw raw;
//        MPU_ReadRaw(mpu, &raw);
//        ax_s += raw.ax; ay_s += raw.ay; az_s += raw.az;
//        gx_s += raw.gx; gy_s += raw.gy; gz_s += raw.gz;
//        HAL_Delay(2);
//    }
//
//    mpu->ax_off = (int16_t)(ax_s / samples);
//    mpu->ay_off = (int16_t)(ay_s / samples);
//    /* Remove 1g from Z — assumes board is flat during calibration */
//    mpu->az_off = (int16_t)(az_s / samples) - (int16_t)(1.0f / mpu->accel_lsb / 9.80665f);
//    mpu->gx_off = (int16_t)(gx_s / samples);
//    mpu->gy_off = (int16_t)(gy_s / samples);
//    mpu->gz_off = (int16_t)(gz_s / samples);
//}

void MPU_Calibrate(MPU_Handle *mpu, uint16_t samples)
{
    int32_t ax_s = 0, ay_s = 0, az_s = 0;
    int32_t gx_s = 0, gy_s = 0, gz_s = 0;

    /* Reset offsets to 0 so we read "true" raw data during the loop */
    mpu->ax_off = mpu->ay_off = mpu->az_off = 0;
    mpu->gx_off = mpu->gy_off = mpu->gz_off = 0;

    for (uint16_t i = 0; i < samples; i++) {
        MPU_Raw raw;
        MPU_ReadRaw(mpu, &raw);

        ax_s += raw.ax; ay_s += raw.ay; az_s += raw.az;
        gx_s += raw.gx; gy_s += raw.gy; gz_s += raw.gz;

        /* 2ms delay gives the 1kHz Output Data Rate (ODR) time to refresh */
        HAL_Delay(2);
    }

    /* Average the samples */
    mpu->ax_off = (int16_t)(ax_s / samples);
    mpu->ay_off = (int16_t)(ay_s / samples);

    /** * CORRECTION: The raw value for 1g depends solely on the LSB sensitivity.
     * If 2g range: 1.0f / (1/16384) = 16384 raw counts.
     * We don't include 9.80665 here because offsets are applied to RAW data.
     */
    int16_t gravity_raw = (int16_t)(1.0f / mpu->accel_lsb);
    mpu->az_off = (int16_t)(az_s / samples) - gravity_raw;

    mpu->gx_off = (int16_t)(gx_s / samples);
    mpu->gy_off = (int16_t)(gy_s / samples);
    mpu->gz_off = (int16_t)(gz_s / samples);
}

void MPU_GetOrientation(MPU_Handle *mpu, MPU_Data *data, MPU_Orientation *ori) {
    uint32_t now = HAL_GetTick();
    float dt = (now - ori->last_time) / 1000.0f; // Convert ms to seconds
    ori->last_time = now;

    // 1. Calculate Pitch/Roll from Accelerometer (The "Truth" for long term)
    // Roll:  atan2(ay, az)
    // Pitch: atan2(-ax, sqrt(ay*ay + az*az))
    float roll_acc = atan2f(data->ay, data->az) * 57.29578f; // Rad to Deg
    float pitch_acc = atan2f(-data->ax, sqrtf(data->ay * data->ay + data->az * data->az)) * 57.29578f;

    // 2. Complementary Filter
    // Alpha is usually between 0.95 and 0.99
    float alpha = 0.98f;

    ori->roll  = alpha * (ori->roll  + data->gx * dt) + (1.0f - alpha) * roll_acc;
    ori->pitch = alpha * (ori->pitch + data->gy * dt) + (1.0f - alpha) * pitch_acc;
}
