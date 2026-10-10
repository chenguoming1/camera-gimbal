# 5 — Camera gimbal application

Project: `6storm32-test`. This replaces the automatic fixed-field hold test with
external-IMU feedback. The earlier behavior remains at tag `test-all-motors-hold`.
The user tested each motor separately and reported all three axes working together
on 2026-10-10. Those settings are now the firmware defaults, released at tag
`camera-gimbal-app`; the CubeIDE build compatibility fix is at
`camera-gimbal-app-build-fix`. No automatic arming occurs on boot/reset. Small-motion testing
is complete; large-angle performance and load/thermal limits are not characterized.

## Hardware and orientation

| Axis | Connector | Phase A | Phase B | Phase C |
| --- | --- | --- | --- | --- |
| Yaw | MOT0 | PA7 / TIM3_CH2 | PB0 / TIM3_CH3 | PB1 / TIM3_CH4 |
| Pitch | MOT1 | PA6 / TIM3_CH1 | PA3 / TIM2_CH4 | PA2 / TIM2_CH3 |
| Roll | MOT2 | PB9 / TIM4_CH4 | PA1 / TIM2_CH2 | PB8 / TIM4_CH3 |

The camera MPU6050 is **I2C1, PB6/PB7, address `0x68`**, detected with the external
sensor attached. The onboard sensor is also on I2C1, at `0x69`, identified earlier
by unplugging the external sensor. The app strictly uses `0x68`; it never substitutes
the fixed onboard sensor when the camera sensor fails. No new CubeMX changes are
required. Keep the existing 72 MHz clock, 48 MHz USB, I2C1 at 100 kHz, and timers
PSC=0 / ARR=3599. TIM2 starts TIM3/TIM4 together at a 20 kHz carrier.

User-confirmed actual wiring: **MOT0=yaw, MOT1=pitch, MOT2=roll**.
The driver routes roll/pitch/yaw control to MOT2/MOT1/MOT0 respectively.
All command arrays and telemetry remain in **roll, pitch, yaw order**. For example,
`enable 1 0 0` selects roll on MOT2; it does not mean connector MOT0.
Status explicitly reports `order=roll,pitch,yaw motor_ports=2,1,0`.

Camera coordinates are **X forward through the lens, Y camera right, Z down**.
Positive angles: roll right side down, pitch lens up, yaw lens right.

The initial photo-derived identity mapping was rejected when lens-up changed the
roll value. Two-pose discovery produced the commissioned mapping **`axes -2 1 3`**:
camera X=-sensor Y, camera Y=sensor X, camera Z=sensor Z. The user confirmed that
right-side-down movement increased roll and completed powered pitch/yaw tests.
The pin-1 dot helps identify package axes; camera mounting and movement checks
establish the final mapping. Startup now has `orientation_ok=1` for this mounting.

Reference: [InvenSense MPU6050 product specification, sections 7.1 and 11.1](https://files.seeedstudio.com/wiki/Xadow_IMU_6DOF/res/MPU6050.pdf).
The dot is a package orientation reference; it does not determine motor wiring
polarity. A motor's direction still needs its own test.

## Startup and control

1. Outputs stay zero while the external IMU initializes.
2. The app collects 400 stationary samples, approximately two seconds. Moving it
   restarts calibration. Keep the camera still; it need not be perfectly level.
3. State becomes `DISARMED`, with all motors selected (`enable=1,1,1`) but all
   PWM outputs zero. Reset never starts the motors.
4. `arm` requires a healthy calibrated IMU, valid orientation, confirmed motor
   directions (saved in this configuration), at least one selected motor with nonzero
   power, PC3 released, and healthy timers.
5. Arming ramps the magnetic field for one second, then enables feedback. A rotor
   can move to a nearby electrical equilibrium during this alignment.
6. Roll/pitch target zero (level); yaw target zero is the relative heading at
   calibration. `hold` captures the current attitude. Commands slew at 10 deg/s
   for roll/pitch and 20 deg/s for yaw.

The MPU runs at 200 Hz, +/-500 deg/s and +/-4 g with its 44/42 Hz low-pass setting.
Only frames with DATA_RDY set are accepted. The quaternion filter integrates gyro
rates and corrects roll/pitch using gravity when acceleration is near 1 g. Gyro
bias is measured during stationary calibration. No magnetometer is present:
**yaw is relative and can drift; this is not absolute compass heading hold**.

Each PID produces electrical field speed, which is integrated into the motor's
three sine phases. This uses empirical gains and does not require entering a pole
count. There is no rotor encoder, current feedback, or measured torque control.
Saved gains are KP=5, KI=0, KD=8 on each axis, with electrical speed limited to
600 deg/s and bounded conditional integration. These values settled during the
user's powered tests; roll previously swung with KP=35, KI=5, KD=8. Large rotations
and nested-axis coupling across the full range are not characterized;
start near level. Absolute roll or pitch exceeding 60 degrees stops the drive.

Green blinking = waiting/calibrating/disarmed/ramping; green solid = ACTIVE.
Red = fault or missing external IMU. Blinking green alone never proves stabilization.

## USB console

Connect the **controller's USB data port** to the computer. ST-LINK's USB port is
for programming/debug and its virtual serial port does not carry these messages.
Open its CDC port with any serial terminal (115200, 8N1), or use:

```bash
python3 scripts/gimbal-console.py /dev/cu.usbmodemXXXX
```

Replace the path with the controller's actual port. Status arrives four times per
second; `angle_mdeg=10000` means 10 degrees. `accel_mg=-1000` means -1 g. Replies
are in the `reply=` field. Commands end with Enter; CR/LF are accepted.

| Command | Effect / restrictions |
| --- | --- |
| `status`, `help` | Read status / command list |
| `stop` | Zero all outputs; remain disarmed; faults stay latched |
| Ctrl-C | Immediate stop from USB receive callback; latches fault |
| `calibrate` | While stopped, recover I2C, clear fault and recalibrate; never arms |
| `axes -2 1 3` | Signed sensor axes for body X/Y/Z; must form a right-handed rotation; recalibrates |
| `level`, then `noseup` | Optional two-pose axis discovery, described below |
| `dir 1 -1 -1` | Roll/pitch/yaw motor signs; each must be -1 or +1; explicitly enables the direction arm gate |
| `enable 1 0 0` | Select only roll/MOT2; array order is roll,pitch,yaw; values 0/1; change while stopped |
| `power 200 200 200` | Differential PWM span in permille, 0..400; change while stopped |
| `gains 0 5 0 8` | Axis 0=roll, 1=pitch, 2=yaw; KP/KI/KD, each 0..100; change while stopped |
| `target 0 10 0` | Roll/pitch/yaw degrees; roll +/-30, pitch +/-45, yaw +/-180 |
| `hold` | Set target to current measured roll/pitch/yaw |
| `center` | Target level roll/pitch, retain current relative yaw |
| `arm` | Explicit ramp/start after all gates pass |

**USB changes are RAM only**. The defaults below are compiled into
`Core/Inc/gimbal_config.h` and restored on every reset. Both orientation and
direction confirmation flags are 1 for the tested camera mounting and wiring.
Startup still calibrates and remains DISARMED until `arm`.

### Saved defaults and normal startup

| Setting | Roll / pitch / yaw value |
| --- | --- |
| Motor ports | MOT2 / MOT1 / MOT0 |
| Sensor axes (camera X/Y/Z) | -2 / 1 / 3 |
| Motor direction | +1 / -1 / -1 |
| Motor selection | 1 / 1 / 1 |
| Differential PWM span | 200 / 200 / 200 permille |
| KP / KI / KD on each axis | 5 / 0 / 8 |

Keep the camera stationary during boot calibration, check `state=DISARMED fault=0`
and `cal=400`, and connect 3S while disarmed. After the supply is connected and any
reset/calibration has completed, send each line separately:

```text
hold
status
arm
```

`hold` captures the current pose; calibration otherwise initializes all targets to
zero. Status should show `axes=-2,1,3 orient=1 dir=1,-1,-1 dir_ok=1`,
`enable=1,1,1 power=200,200,200`. Keep movement small and within the frame/cable
clearance; send `stop` if an axis drives away or oscillation grows. The +/-60 degree
tilt cutoff applies to roll/pitch, not yaw, and is not a mechanical travel limit.

### Check orientation with 3S disconnected

After calibration, manually move the camera and read the angles:

- Lens up: pitch increases.
- Camera right side down: roll increases.
- Lens right when viewed from above: yaw increases.

If the sensor mounting changes, use two-pose discovery before powered testing:

1. Hold the camera level and still for one second, then send `level`.
2. Keep roll level, tilt the lens **up** 30–45 degrees, hold still for one second,
   then send `noseup`.
3. Firmware reports signed axes and recalibrates. Check all three angle signs.

This assumes the sensor is mounted parallel to the camera with axis directions
aligned to its sides. It does not estimate an arbitrary angled mounting.

### Powered commissioning, one motor at a time

Finish orientation checks first. Use Run/free-running firmware when 3S is connected;
do not halt or attach SWD while motors are powered. Balance the camera mechanically,
keep cable travel clear, and have the battery connector accessible.

1. With the controller running and disarmed, send `enable 1 0 0` for roll on MOT2.
2. Check saved `dir=1,-1,-1`, send `power 200 200 200`, then `hold` to capture the pose.
3. Connect 3S, then send `arm`. Check roll correction with a small disturbance.
   If it drives away from the target or oscillates, `stop` immediately.
4. If wiring changes, stop and verify the affected motor direction before tuning
   gains. Do not treat the saved signs as verified for a different assembly.
5. Repeat with `enable 0 1 0` for pitch/MOT1, then `enable 0 0 1` for yaw/MOT0, changing
   only the corresponding direction sign if correction is reversed.
6. Once each axis is stable, send `stop`, `enable 1 1 1`, `hold`, then `arm`.
   Test small base movements first; target commands specify absolute angles.

The saved 200 permille is a 20% differential PWM span, **not** a current or torque
limit. Reduce it if heating/current is excessive. Commands are capped at 400.
Do not leave initial trials unattended. If USB is unavailable, PC3 BUT/JP10 to
GND stops all outputs. Releasing PC3 does not restart them.

## Stops and fault recovery

- PC3, debugger `gimbal.request_stop=1`, CPU faults, and USB Ctrl-C stop all phases.
- An I2C read error stops immediately; 25 ms without fresh samples stops the drive.
- The 1 ms SysTick checks freshness and timer configuration independently of the
  main loop. An independent watchdog resets after about 0.5 s at nominal LSI if
  the main loop stalls. It freezes during debugger halts, so **SWD is motor-power-off only**.
- Ring overflow stops immediately and drops input through the next newline.
  Oversized/invalid command lines are rejected, including their tails.
- Faults do not rearm automatically. Release PC3, send Enter after Ctrl-C/overflow,
  then `calibrate`, wait for `DISARMED`, and issue a new `arm` when ready.
  CPU exception handlers do not return; those need a reset (the watchdog also resets).

Fault IDs: 0=OK, 1=stop, 2=button, 3=I2C, 4=stale data, 5=timing/filter,
6=timer, 7=tilt, 8=USB overflow, 9=CPU exception.

Stopping writes zero to all nine IN/PWM compares and forces timer updates. With
DRV8313 ENx high, all INx low means electrical braking, **not high impedance**.
There is no confirmed MCU control of driver ENx/nSLEEP/nRESET on this board.
Disconnecting 3S is the physical motor-power removal.

## Code map

| File | Responsibility |
| --- | --- |
| `Core/Src/gimbal_app.c` | External MPU driver/recovery, calibration, state machine, USB parser, timing, LEDs, watchdog |
| `Core/Src/gimbal_control.c` | Axis mapping, quaternion gravity fusion, PID phase integration, sine PWM math |
| `Core/Src/gimbal_motor.c` | Verified timer start/motor mapping, PWM writes and atomic stop interlock |
| `Core/Inc/gimbal_config.h` | Editable startup configuration and tuning defaults |
| `Core/Src/main.c` | CubeMX initialization and app calls in USER CODE sections |
| `USB_DEVICE/App/usbd_cdc_if.c` | Forward received CDC bytes to bounded application buffer |
| `scripts/test-gimbal.py` | Compile actual firmware modules with host HAL mocks |
| `scripts/test-gimbal-build.py` | Verify build-hook linking before/after CubeIDE regenerates object lists |
| `scripts/debug-gimbal.sh` | Build/flash/live SWD verification or inspect/probe |
| `scripts/gimbal-console.py` | Standard-library USB CDC terminal |

The `makefile.defs` hook adds new modules to the existing generated command-line
build without editing generated source lists. CubeIDE also discovers the new
`.c` files normally; the hook filters duplicates after regeneration.

## Reproduce verification

With motor power disconnected:

```bash
python3 scripts/test-gimbal.py
python3 scripts/test-gimbal-build.py
scripts/debug-gimbal.sh verify
# Later, without flashing (requires matching current ELF):
scripts/debug-gimbal.sh inspect
# Sensor address probe; also disconnect controller USB data for this mode:
scripts/debug-gimbal.sh probe
```

The historical script entry points still dispatch correctly for the current app;
use historical tags when reproducing earlier fixed-field test behavior.

### Initial verification — 2026-10-10 (before powered commissioning)

- Host checks passed: tilt/yaw fusion and gravity correction, mapping validation
  and two-pose discovery, PID saturation, stationary calibration, fragmented and
  invalid commands, arm gates, all nine phase mappings, stops, and fault latching
  for sensor errors, stale samples/control stalls, timers, PC3 and receive overflow.
- ARM build passed without warnings: text=83792, data=860, BSS=8660 bytes for
  the verified I2C-recovery build.
- Initial reset encountered I2C error `0x20` (timeout). Added bounded open-drain
  bus clocks and STOP before sensor initialization; active drive never attempts
  automatic bus recovery. Startup retry remains disarmed.
- Live external MPU6050: address `0x68`, WHO_AM_I `0x68`; samples **1 -> 1001** over
  five seconds, errors **0**, stationary calibration **400** samples.
- At the sampled pose: acceleration approximately `[-300,+333,-887] mg`, angles
  `[-20.7,-17.7,-0.03] degrees`; these describe the pose at that time, not a level
  calibration. Gyro bias approximately `[-3.338,-0.445,+1.022] deg/s`.
- Measured sample/filter processing: **2102 us**, maximum **2205 us** in that run.
- TIM2/TIM3/TIM4 CCER=`0x1110/0x1111/0x1100`, both slave SMCR=`0x16`.
- Startup stayed DISARMED; all nine compare registers stayed zero. A stop request
  latched FAULT/STOP and all nine remained zero. Final reset left firmware running
  with no automatic motor start.
- A subsequent attach compared flash `.isr_vector` (484 bytes), `.text` (79976
  bytes), and `.rodata` (3316 bytes) against the matching ELF: all matched. It saw
  2491 samples with zero errors before intentionally stopping; then reset/detached.
- Log directory for this run:
  `/var/folders/hh/c4mnfzb935q8wsyvzw6tjvl80000gq/T/storm32-imu-debug.qaLJ9T`.
- At this initial stage, physical motor direction, powered stability/torque/current,
  tuned PID gains, watchdog reset under an injected stall, and USB transport were
  unverified. Later USB and powered commissioning findings are recorded below.

ELRS/CRSF receiver control, ESP8266 networking, flash-stored settings and a desktop
GUI are not implemented in this build. This application currently uses USB target
commands and the external camera IMU for stabilization.

### Commissioning finding — 2026-10-10

The user observed the top/yaw motor moving during `enable 1 0 0`. Captured status
confirmed `enable=1,0,0`, `axes=1,2,3`, `dir=-1,1,1`, zero I2C errors and a latched
STOP fault. Motor connection/physical-axis correspondence needs verification;
changing a direction sign cannot resolve a motor assigned to the wrong axis.
The user then confirmed actual wiring: **MOT0=yaw, MOT1=pitch, MOT2=roll**.
The driver now routes roll/pitch/yaw to MOT2/MOT1/MOT0; no cable swaps are required.
The reported lens-up movement changing the first attitude value also needs an IMU
axis check before feedback is armed again. Startup orientation confirmation is
cleared until a successful mounting-axis configuration.

At this intermediate stage, startup selection was `enable=0,0,0`. Arming with no selected motor is
rejected, and direction/selection commands report explicit axis values and connectors. Host checks
cover no-selection arm rejection and zero PWM on both unselected motors during
a roll-only run. Tests also check pitch-only and yaw-only routing independently.
The corrected build was flashed with 3S disconnected after the user confirmed
USB/ST-LINK remained connected. Verification passed:

- ARM build without warnings: text=84216, data=860, BSS=8660 bytes.
- External samples 1 -> 999 over five seconds, zero errors, calibration complete.
- Default selection `enable=0,0,0`, `orientation_ok=0`, `direction_ok=0`.
- With motor power off, a driver-only SWD bench call used distinct strengths:
  roll=100, pitch=200, yaw=300 permille, all electrical phases zero. Actual CCRs:
  MOT2/roll `[1800,1955,1644]`, MOT1/pitch `[1800,2111,1488]`,
  MOT0/yaw `[1800,2267,1332]`. This verified register routing, not powered motion.
- The subsequent ISR stop cleared all nine previously nonzero compares.
- A fresh attach compared flash sections against the ELF successfully, then
  reset/detached. Firmware was left running disarmed with no selected motors.
- Logs: `/var/folders/hh/c4mnfzb935q8wsyvzw6tjvl80000gq/T/storm32-imu-debug.8rGpdJ`
  and `/var/folders/hh/c4mnfzb935q8wsyvzw6tjvl80000gq/T/storm32-imu-debug.K6PTQ2`.

### Powered commissioning and saved defaults — 2026-10-10

- USB CDC commands and live status were exercised by the user throughout testing.
- Runtime mounting axes became `-2,1,3`; right-side-down motion increased the first
  angle, confirming roll's observed sign. All telemetry/commands use roll,pitch,yaw.
- Roll/MOT2 drove away with direction -1. Direction +1 and gains 5/0/8 settled;
  the user reported a small shake when disturbing it, so further tuning remains.
- Pitch/MOT1 drove backward by itself with direction +1 and reached the 60 degree
  cutoff (`fault=7`). Direction -1 then settled with gains 5/0/8.
- Yaw/MOT0 rotated continuously with direction +1; direction -1 held during the
  user's test. Yaw remains relative and subject to gyro drift.
- The user reported all three axes together looked good at power 200 each and
  requested these values as firmware defaults. All three are now selected at boot,
  with orientation/directions confirmed; automatic arming remains disabled.
- Recalibration clears a latched tilt fault while stopped. Angles freeze in FAULT
  even while raw IMU readings continue; `reply` may retain an earlier command's
  response (for example `Armed`) after a fault. Use `state` and `fault` as authority.
- Host regression checks passed for saved defaults, disarmed zero-output startup,
  explicit all-axis arming, no-selection/orientation/direction gates, single-axis
  isolation, phase routing, command handling and emergency stops.
- Final defaults ARM build passed without warnings: text=84256, data=860,
  BSS=8660 bytes. Python scripts parsed successfully, shell scripts passed
  `bash -n`, and the CRLF-aware whitespace check passed. This release does not
  flash the connected board; load the rebuilt ELF with motor power off.
- Release: `camera-gimbal-app` on `main`, authored as Aung Bo Naing
  `<chenguoming1@gmail.com>`. The release includes application modules, saved
  configuration, host checks, USB console, debug scripts and these notes.

The powered results above are user observations, not measured torque/current,
thermal limits or full-range stabilization qualification. Settings are specific to
this camera mounting and motor wiring.


### CubeIDE build compatibility fix — 2026-10-10

After CubeIDE regenerated `Debug/Core/Src/subdir.mk` and `objects.list`, linking
failed with multiple definitions of `gimbal`, `Gimbal_Init`, and other gimbal
symbols. The generated paths were `./Core/Src/gimbal_*.o`, while `makefile.defs`
compared them with `Core/Src/gimbal_*.o`. It therefore added the same modules again
through `USER_OBJS`. The initial command-line build had used the older object list
without those modules, so it did not expose this problem.

The hook now normalizes the leading `./` on existing `OBJS` and `USER_OBJS` before
adding missing modules. Generated CubeIDE build files do not need manual edits.
The actual regenerated build now passes: text=84256, data=860, BSS=8660 bytes.
`python3 scripts/test-gimbal-build.py` passed five cases: missing modules, bare
paths, CubeIDE-prefixed paths, mixed/partial lists and existing user objects.

Release: follow-up commit and tag `camera-gimbal-app-build-fix`. To rebuild in
CubeIDE, use **Project > Build Project**. No firmware behavior or motor settings
changed, and this fix does not flash the board.
