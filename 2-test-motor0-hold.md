# 2 — MOT0 roll hold test

Git tag: `test-motor0-hold`.

This document describes that tagged checkpoint. The current three-motor version
is documented in [4-test-all-motors-hold.md](4-test-all-motors-hold.md).

Project: `6storm32-test`. Test one motor before adding yaw movement or additional
axes. The connected board has DRV8313 drivers and the user uses a 3S motor supply.
The IMU runs independently and does not control this test.

## Existing projects and drive strength

| Project | Configured strength | Source |
| --- | --- | --- |
| `2-DRV8313-test` | 40% | `Core/Src/motor.h`, `HOLD_STRENGTH` |
| `4-gimbal-v1` | 45% | `Core/Inc/gimbal.h`, `GIMBAL_STRENGTH` |
| `4-gimbal-yaw` | 45% | `Core/Src/main.c`, `MOTOR_STRENGTH` |
| `5-gimbal-brugi` | Pitch 65%, roll 70%, yaw 45% | `Core/Src/brugi_config.c` |

These are configured PWM strengths, not measured torque/current limits or proof
of successful operation. The earlier new test's 6% was much weaker than these
settings. The one-motor test now uses **40%**, matching `2-DRV8313-test`, with a
one-second ramp. It does not copy the higher Brugi settings.

The old project shifts sine waves into 0–40% duty. The new test centers its
waveforms at 50%, with the same average differential command. At fixed electrical
angle zero, duties are approximately 50%, 67.3%, and 32.7%.

## Output and behavior

Only **MOT0** is active:

| Phase | MCU pin | PWM channel |
| --- | --- | --- |
| A | PA7 | TIM3_CH2 |
| B | PB0 | TIM3_CH3 |
| C | PB1 | TIM3_CH4 |

The electrical field angle stays fixed; there is no sweep or yaw movement.
TIM3 runs independently at 20 kHz (72 MHz / 3600), with PSC 0 and ARR 3599.
TIM3_CH1 stays disabled. TIM2/TIM4 stay stopped with their output channels disabled.
MOT1/MOT2 inputs PA1/PA2/PA3/PA6/PB8/PB9 are configured as push-pull GPIO outputs
and driven low. No extra CubeMX configuration is required.

After motor initialization:

1. Wait two seconds at zero compare values.
2. Ramp MOT0's differential strength from zero to 40% over one second.
3. Keep that fixed field until explicitly stopped.

The rotor may move to a nearby magnetic equilibrium during alignment. This is
open-loop holding torque, not measurement/preservation of the exact initial
shaft angle. It does not stabilize a camera or calculate orientation.

## Run the physical test

1. With motor power disconnected, connect **one motor to MOT0**. Other motors
   may be unplugged to make the test easier to observe.
2. Flash the current `6storm32-test` if needed. Resume if using Debug.
3. Connect the motor supply and reset the MCU to start the automatic sequence.
   Expect alignment/holding after approximately three seconds.
4. Gently check whether MOT0 resists displacement; do not force it or hold it
   stalled. Check current and temperature during a brief first observation.
5. Disconnect motor power when finished, or set `motor_test.request_stop = 1`
   while running. PC3 to GND on BUT/JP10 also stops the drive.

**Do not pause the debugger while motor power is connected.** PWM can retain its
last command when the software stop logic is paused. Reset restarts the test.
Holding PC3 low cancels the automatic startup. After stopping, releasing PC3
alone does not restart it; use reset or `motor_test.request_run = 1`.

40% is not a current limit. The battery voltage, winding resistance, and load
still determine current and heating. Stop/remove motor power if current or
heating is excessive. The earlier project setting is not a thermal rating.

## DRV8313 readiness

This uses the DRV8313's supported three-PWM drive on IN1/IN2/IN3. EN1/EN2/EN3,
nSLEEP, and nRESET must be high. VM must be at least 8 V and within the actual
board's rating. nFAULT, with a pull-up, should be high when healthy.
See [TI's DRV8313 data sheet](https://www.ti.com/lit/ds/symlink/drv8313.pdf).

The firmware does not invent GPIO assignments for those control/fault signals.
Some v1.31 boards hardwire enables high, but that wiring on this board still
needs physical checking. A 3S battery connected elsewhere does not establish VM
at the driver. A healthy IMU/green LED also does not establish motor power.

All-low IN signals with EN high produce electrical braking, not static angle
holding or high impedance. Older project comments saying the DRV8313 can only
source current, or calling zero compares “coast,” are incorrect.

## Debug values

Inspect `motor_test` in Live Expressions while running:

| State | Meaning |
| --- | --- |
| 0 | Two-second startup delay |
| 1 | One-second ramp |
| 2 | Continuous fixed-field hold |
| 3 | Stopped until reset or a new request |
| 4 | Latched timer/CPU fault |

| Stop reason | Meaning |
| --- | --- |
| 0 | No stop/current run |
| 1 | PC3 pressed |
| 2 | Explicit `request_stop` |
| 3 | Timer configuration, start, counter, or output-enable failure |
| 4 | CPU fault/emergency stop |

At full hold expect `state=2`, `drive_permille=400`, `compare=[1800,2423,1177]`,
`timer_cr1 & 1 = 1`, and `timer_ccer=0x1110`. Compare values are TIM3 CH2/3/4.
`runs` counts starts. `elapsed_ms` excludes the startup delay.

The firmware monitors TIM3 CEN and channel-enable bits and stops on a failure.
Stops write zero compares and force an update even if the counter is stopped.
Fault handlers perform the same motor stop. These checks do not detect missing
motor supply, incorrect board routing, driver faults, or inadequate winding current.

Green is still the IMU heartbeat. USB CDC reports now append `motor=0`,
`motor_state`, `motor_stop`, `motor_run`, `motor_ms`, and `drive_permille`.
The previous `yaw_test` and `motor_compare` diagnostics were replaced by
`motor_test` for this single-motor test.

## Host and SWD scripts

Run logic checks without hardware:

```bash
python3 scripts/test-motor-hold.py
```

This compiles the actual firmware functions with HAL mocks. It checks that only
MOT0 PWM starts, inactive motor inputs are low, TIM3 has no trigger dependency,
the 40% ramp/hold persists, and stops, reruns, startup inhibition, timer failures,
CPU faults, tick rollover, and IMU independence behave as intended.

To inspect actual register values, first **disconnect the 3S battery**, leave
MCU/ST-LINK power connected, and end any CubeIDE debug session:

```bash
scripts/debug-onboard-imu.sh motor
```

This mode attaches without flashing. It compares flash bytes with the local ELF,
checks the live MOT0 hold and inactive outputs, then requests a stop and leaves
the application running in MOTOR_STOPPED. It may request a new hold if the test
was already stopped. Reset the board to rerun after reconnecting motor power.

The `probe` and `verify` IMU modes remain available with motor power disconnected.
Their GDB commands now use `Motor_TestInit` and `motor_test.request_stop` to
inhibit this automatic motor startup during IMU diagnostics.

## Debug evidence — 2026-10-09

Before changing to MOT0 only, flash matched the latest three-motor build. Actual
TIM2/3/4 CEN/channel-enable bits were set and compare values matched the 6% hold
commands. That ruled out stale firmware and stopped PWM timers; it did not prove
motor supply, driver enable/fault status, or holding torque.

The MOT0-only 40% firmware built and passed the host logic checks. It was flashed
and byte-verified with the motor battery disconnected. Live SWD checks passed:

- TIM3 CR1=`0x1`, CCER=`0x1110`, compares=`[1800,2423,1177]` at full hold.
- TIM2/TIM4 counters and channel enables were zero.
- MOT0 pins were alternate-function outputs; MOT1/MOT2 pins were GPIO outputs low.
- The explicit stop cleared all MOT0 compares. The application was left running
  in `MOTOR_STOPPED`; reset is needed for the next automatic test.

Powered holding torque/current/temperature still require the physical test above;
no motor was energized during these SWD checks.
