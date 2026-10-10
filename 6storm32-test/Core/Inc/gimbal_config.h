#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H
/* Camera body: X forward through lens, Y camera right, Z down.
 * Verified on this gimbal through USB commissioning on 2026-10-10:
 * camera X=-sensor Y, camera Y=sensor X, camera Z=sensor Z.
 * All arrays use roll, pitch, yaw order. USB changes remain RAM-only.
 */
#define GIMBAL_DEFAULT_AXES {-2, 1, 3}
#define GIMBAL_ORIENTATION_CONFIGURED 1U
/* Actual motors: MOT0=yaw, MOT1=pitch, MOT2=roll.
 * Directions and gains were tested individually, then with all axes active.
 * Selection does not arm: boot calibrates and stays DISARMED until USB arm.
 */
#define GIMBAL_DEFAULT_DIRECTIONS {1, -1, -1}
#define GIMBAL_DIRECTIONS_CONFIGURED 1U
#define GIMBAL_DEFAULT_ENABLED {1, 1, 1}
#define GIMBAL_DEFAULT_POWER {200, 200, 200}
#define GIMBAL_DEFAULT_KP 5.0f
#define GIMBAL_DEFAULT_KI 0.0f
#define GIMBAL_DEFAULT_KD 8.0f
#endif
