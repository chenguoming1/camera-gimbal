# 4 — Hold all three motors

Git tag: `test-all-motors-hold`.

Project: `6storm32-test`. This extends the earlier
[MOT0 test](2-test-motor0-hold.md) and [two-motor test](3-test-motor1-hold.md).

## Mapping and behavior

| Motor / axis | Phase A | Phase B | Phase C |
| --- | --- | --- | --- |
| MOT0 / roll | PA7, TIM3_CH2 | PB0, TIM3_CH3 | PB1, TIM3_CH4 |
| MOT1 / pitch | PA6, TIM3_CH1 | PA3, TIM2_CH4 | PA2, TIM2_CH3 |
| MOT2 / yaw | PB9, TIM4_CH4 | PA1, TIM2_CH2 | PB8, TIM4_CH3 |

All three motors receive **40% differential PWM strength** at a fixed electrical
angle. After reset: two seconds at zero compare values, a one-second ramp,
then continuous hold until stopped. There is no commanded rotation or IMU feedback.
Each rotor may move to a nearby magnetic equilibrium during alignment; this does
not measure or guarantee preservation of the exact initial shaft position.

TIM2 is the master. TIM3 and TIM4 use ITR1/trigger mode and start from TIM2's
enable trigger. All three timers use PSC=0, ARR=3599 (20 kHz at 72 MHz timer clock).
Counters are aligned before each run. All nine motor pins are explicitly
configured as alternate-function push-pull outputs at 2 MHz GPIO speed.
No additional CubeMX setup is required.

## Physical test

1. With 3S disconnected, connect roll to MOT0, pitch to MOT1, yaw to MOT2.
2. Flash this build if needed; leave firmware running.
3. Reconnect 3S and press Reset. Wait about three seconds for the ramp to finish.
4. Briefly check all three motors resist gentle movement. Monitor current and
   heating; disconnect motor power if either is excessive. 40% is not a current limit.
5. PC3 to GND (BUT/JP10), or `motor_test.request_stop=1` while running, stops
   all three together. Reset or `motor_test.request_run=1` reruns after a normal stop.
   Releasing PC3 alone does not restart the drive.

Do not pause the debugger with motor power connected: PWM can retain its last
command while software stop checks are paused. The existing DRV8313 enable,
sleep/reset, motor supply, and wiring requirements still apply.

## Diagnostics

At full hold:

- `motor_test.state=MOTOR_HOLD`, `drive_permille=400`.
- `compare` (MOT0), `compare_motor1`, `compare_motor2`: `[1800,2423,1177]`.
- TIM2: CEN=1, CCER=`0x1110`, SMCR=0, CR2 MMS=`0x10`.
- TIM3: CEN=1, CCER=`0x1111`, SMCR=`0x16`.
- TIM4: CEN=1, CCER=`0x1100`, SMCR=`0x16`.
- Actual yaw compares: TIM4 CCR4=1800, TIM2 CCR2=2423, TIM4 CCR3=1177.
- GPIO input readback should observe high/low on PA1/2/3/6/7 and PB0/1/8/9.

The common stop/fault path clears all nine phase compares and forces updates on
all three timers. Timer checks cover all counters/channel enables and both slave
configurations. USB CDC reports `motors=0,1,2`. Green remains the IMU heartbeat.

Host logic checks:

```bash
python3 scripts/test-motor-hold.py
```

Live SWD checks, **with 3S disconnected and ST-LINK connected**:

```bash
scripts/debug-onboard-imu.sh motor
```

The script verifies flash against the ELF, all motor compares, timer PWM modes,
GPIO configuration/remapping, optional pin input-readback levels, and the common
stop. It leaves firmware running with all three motor groups stopped. Input
readback does not measure exact PWM frequency/duty or verify driver outputs,
winding current, temperature, or holding torque.

## Verification — 2026-10-10

- ARM Debug build and host logic checks passed, including all nine PWM-start
  failure cases, stop/rerun, counter/channel/slave faults, and tick rollover.
- Flashed and verified with 3S disconnected, as confirmed by the user.
- Live SWD flash comparison, all three timer/compare configurations, PWM mode,
  and GPIO/remapping checks passed.
- Both logic levels were observed on all nine active MCU pins: PA high/low masks
  `0xce/0xce`, PB masks `0x303/0x303`.
- Explicit stop cleared all nine active compares. Firmware was left running in
  `MOTOR_STOPPED`; reset after reconnecting 3S to start the physical test.
- Powered holding torque/current/temperature have not been verified.
