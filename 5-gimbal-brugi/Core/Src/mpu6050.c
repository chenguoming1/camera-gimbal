/* =============================================================
 * mpu6050.c
 * MPU6050 driver — dual device support
 * =============================================================*/
#include "mpu6050.h"
#include <math.h>

/* ---------- private helpers -------------------------------- */

static void mpu_write(MPU_Handle *mpu, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    HAL_I2C_Master_Transmit(mpu->hi2c, mpu->addr << 1, buf, 2, 10);
}

static uint8_t mpu_read_byte(MPU_Handle *mpu, uint8_t reg)
{
    uint8_t val = 0;
    HAL_I2C_Master_Transmit(mpu->hi2c, mpu->addr << 1, &reg, 1, 10);
    HAL_I2C_Master_Receive(mpu->hi2c,  mpu->addr << 1, &val, 1, 10);
    return val;
}

static void mpu_read_burst(MPU_Handle *mpu, uint8_t reg,
                           uint8_t *buf, uint16_t len)
{
    HAL_I2C_Master_Transmit(mpu->hi2c, mpu->addr << 1, &reg, 1, 10);
    HAL_I2C_Master_Receive(mpu->hi2c,  mpu->addr << 1, buf, len, 20);
}

/* ---------- public API ------------------------------------- */

uint8_t MPU_Init(MPU_Handle *mpu, I2C_HandleTypeDef *hi2c, uint8_t addr,
                 MPU_GyroRange gyro_range, MPU_AccelRange accel_range)
{
    mpu->hi2c = hi2c;
    mpu->addr = addr;
    mpu->ax_off = mpu->ay_off = mpu->az_off = 0;
    mpu->gx_off = mpu->gy_off = mpu->gz_off = 0;

    HAL_Delay(100);

    /* Verify device identity */
    uint8_t who = mpu_read_byte(mpu, MPU_REG_WHO_AM_I);
    if (who != 0x68 && who != 0x69 && who != 0x70) return 0; /* not found */

    /* Wake up: clear sleep bit, use PLL with X gyro as clock source */
    mpu_write(mpu, MPU_REG_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    /* Sample rate divider: 0 → 1 kHz / (1+0) = 1 kHz */
    mpu_write(mpu, MPU_REG_SMPLRT_DIV, 0x00);

    /* DLPF: bandwidth ~94 Hz accel, 98 Hz gyro */
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

    return 1; /* success */
}

void MPU_ReadRaw(MPU_Handle *mpu, MPU_Raw *raw)
{
    uint8_t buf[14];
    mpu_read_burst(mpu, MPU_REG_ACCEL_XOUT_H, buf, 14);

    raw->ax   = (int16_t)((buf[0]  << 8) | buf[1]);
    raw->ay   = (int16_t)((buf[2]  << 8) | buf[3]);
    raw->az   = (int16_t)((buf[4]  << 8) | buf[5]);
    raw->temp = (int16_t)((buf[6]  << 8) | buf[7]);
    raw->gx   = (int16_t)((buf[8]  << 8) | buf[9]);
    raw->gy   = (int16_t)((buf[10] << 8) | buf[11]);
    raw->gz   = (int16_t)((buf[12] << 8) | buf[13]);

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

    /* Convert to physical units */
    data->ax = raw.ax * mpu->accel_lsb * 9.80665f; /* m/s² */
    data->ay = raw.ay * mpu->accel_lsb * 9.80665f;
    data->az = raw.az * mpu->accel_lsb * 9.80665f;
    data->gx = raw.gx * mpu->gyro_lsb;  /* deg/s */
    data->gy = raw.gy * mpu->gyro_lsb;
    data->gz = raw.gz * mpu->gyro_lsb;
}

void MPU_Calibrate(MPU_Handle *mpu, uint16_t samples)
{
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

    /* Clear existing offsets first */
    mpu->ax_off = mpu->ay_off = mpu->az_off = 0;
    mpu->gx_off = mpu->gy_off = mpu->gz_off = 0;

    for (uint16_t i = 0; i < samples; i++) {
        MPU_Raw raw;
        MPU_ReadRaw(mpu, &raw);
        ax_sum += raw.ax;
        ay_sum += raw.ay;
        az_sum += raw.az;
        gx_sum += raw.gx;
        gy_sum += raw.gy;
        gz_sum += raw.gz;
        HAL_Delay(2);
    }

    mpu->ax_off = (int16_t)(ax_sum / samples);
    mpu->ay_off = (int16_t)(ay_sum / samples);
    /* az bias: remove 1g (gravity) from Z — assumes flat mount */
    mpu->az_off = (int16_t)(az_sum / samples) - (int16_t)(1.0f / mpu->accel_lsb / 9.80665f);
    mpu->gx_off = (int16_t)(gx_sum / samples);
    mpu->gy_off = (int16_t)(gy_sum / samples);
    mpu->gz_off = (int16_t)(gz_sum / samples);
}
