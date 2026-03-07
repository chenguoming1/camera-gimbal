/* =============================================================
 * imu.h
 * Angle estimation — complementary filter over two MPU6050s
 *
 * Both IMUs share I2C1 (PB6/PB7):
 * Board IMU  : address 0x69 (AD0 HIGH) — measures board/frame orientation
 * Camera IMU : address 0x68 (AD0 LOW)  — measures camera head orientation
 *
 * Error = camera_angle - board_angle
 * The gimbal drives motors to keep Error = 0 (horizon lock).
 * =============================================================*/
#ifndef IMU_H
#define IMU_H

#include "mpu6050.h"

/* Complementary filter coefficient (0..1).
 * Higher → trusts gyro more (smoother but drifts).
 * Lower  → trusts accel more (absolute but noisy).  */
#define CF_ALPHA   0.98f

/* Time step in seconds — must match your control loop rate */
#define IMU_DT     0.001f   /* 1 kHz */

/* Three-axis angle state (degrees) */
typedef struct {
    float roll;   /* rotation around X */
    float pitch;  /* rotation around Y */
    float yaw;    /* rotation around Z (gyro-only, no mag) */
} IMU_Angles;

/* Full IMU system state */
typedef struct {
    MPU_Handle   board;       /* I2C1, addr 0x69 */
    MPU_Handle   cam;         /* I2C2, addr 0x68 */
    IMU_Angles   board_angle; /* absolute frame angles    */
    IMU_Angles   cam_angle;   /* absolute camera angles   */
    IMU_Angles   error;       /* cam_angle - board_angle  */
} IMU_State;

/* API */
uint8_t IMU_Init(IMU_State *imu,
                 I2C_HandleTypeDef *hi2c);  /* both IMUs share I2C1 */
void    IMU_Calibrate(IMU_State *imu);
void    IMU_Update(IMU_State *imu);   /* call every DT seconds */

#endif /* IMU_H */
