# 3 — Add MOT1 pitch hold

Project: `6storm32-test`. This extends the tagged MOT0-only test described in
[2-test-motor0-hold.md](2-test-motor0-hold.md). MOT0 and MOT1 now hold together;
MOT2 stays inactive in this earlier stage. The current all-three-motor version
is documented in [4-test-all-motors-hold.md](4-test-all-motors-hold.md).

## Outputs

| Motor port / axis | Phase A | Phase B | Phase C |
| --- | --- | --- | --- |
| MOT0 / roll | PA7, TIM3_CH2 | PB0, TIM3_CH3 | PB1, TIM3_CH4 |
| MOT1 / pitch | PA6, TIM3_CH1 | PA3, TIM2_CH4 | PA2, TIM2_CH3 |
| MOT2 / yaw (inactive) | PB9, TIM4_CH4 | PA1, TIM2_CH2 | PB8, TIM4_CH3 |

The agreed mapping for this project is **MOT0=roll, MOT1=pitch, MOT2=yaw**.
Connect the motors to match these assignments. This fixed-field test does not
use axis feedback.

Both use the same fixed electrical angle and **40% differential PWM strength**.
After reset: two seconds at zero output, one-second ramp, then continuous hold.
The rotor can move to a nearby magnetic equilibrium. There is no angle feedback,
so this does not guarantee preservation of the exact starting shaft position.
The IMU does not control either motor.

MOT1 spans two timers. TIM2 is the master with enable as TRGO; TIM3 is armed in
trigger mode using ITR1 from TIM2. Both use PSC=0 and ARR=3599 (20 kHz), and
counters are aligned before starting/rerunning. STM32 HAL leaves the slave
counter disabled until the hardware trigger arrives. See ST's
[RM0008 reference manual](https://www.st.com/resource/en/reference_manual/cd00171190.pdf)
and the local `HAL_TIM_PWM_Start` implementation.

MOT2's PA1/PB8/PB9 inputs remain GPIO outputs low; TIM2_CH2 stays disabled,
and TIM4 stays stopped with all outputs disabled. Existing CubeMX PWM pin
configuration already covers MOT1; no CubeMX changes are needed.

## Test

1. With 3S disconnected, connect the roll motor to MOT0 and the pitch motor to MOT1.
2. Flash this build if needed. Keep the application running.
3. Reconnect 3S and press Reset. Wait about three seconds for both motors to hold.
4. Briefly check resistance to gentle movement and watch temperature/current.
   Disconnect motor power if heating or current is excessive; 40% is not a current limit.
5. Stop both using PC3 to GND (BUT/JP10), or set `motor_test.request_stop=1`
   while firmware is running. Reset reruns automatically; releasing PC3 alone
   does not restart. `motor_test.request_run=1` also reruns after a normal stop.

Do not halt the debugger with 3S connected: timer outputs can retain their last
command while software stop checks are paused.

## Diagnostics and verification

At full hold, `motor_test.state=MOTOR_HOLD`, `drive_permille=400`:

- `compare` (MOT0) and `compare_motor1` (MOT1): `[1800,2423,1177]`.
- TIM3: CEN=1, CCER=`0x1111`, SMCR=`0x16`.
- TIM2: CEN=1, CCER=`0x1100`, CR2 MMS=`0x10`, SMCR=0.
- Actual MOT1 CCRs: TIM3 CCR1=1800, TIM2 CCR4=2423, TIM2 CCR3=1177.
- MOT2: TIM2 CCR2=0, TIM4 CEN/CCER=0, PA1/PB8/PB9 GPIO outputs low.

Timer failure checks cover both timers and synchronization configuration; stop
and CPU fault handlers clear all six active phase compares. USB CDC reports
`motors=0,1`. Green remains the IMU heartbeat and is not proof of holding torque.

Host checks (no hardware):

```bash
python3 scripts/test-motor-hold.py
```

Live SWD checks, **with 3S disconnected and ST-LINK connected**:

```bash
scripts/debug-onboard-imu.sh motor
```

The script checks flash matches the local ELF, checks both motor outputs and
MOT2 isolation, requests stop, and leaves firmware running with both motors
stopped. Reset after reconnecting 3S to start the physical test.

### Results — 2026-10-10

- ARM Debug build and host logic checks passed, including all six PWM-start
  failures, faults on either active timer, stop/rerun, and synchronization checks.
- Flashed and verified with the user-confirmed 3S supply disconnected.
- Live SWD matched flash against the ELF and verified both sets of compare
  registers, TIM2/TIM3 configuration, and inactive MOT2 pins/timer.
- Verified a rerun reached full hold, then explicit stop cleared all six active
  compares. Firmware was left running in `MOTOR_STOPPED` with both motors stopped.
- Actual powered holding torque/current/temperature have not been tested.

### Holding investigation — 2026-10-10

The user reported MOT0 and MOT2 holding with 3S connected, despite this firmware
commanding MOT0/MOT1 pin groups. Board connector labels and actual motor wiring
still need confirmation; no connector reassignment has been made from that
observation alone.

With 3S disconnected, fresh SWD inspection matched flash to the ELF and found:

- MOT0/MOT1 full-hold compares and timer/GPIO configuration as expected.
- TIM3 CCMR1/CCMR2 and TIM2 CCMR2 all `0x6868`: PWM mode 1 with preloads enabled.
- AFIO MAPR `0x04000000`: no TIM2/TIM3 pin remapping.
- MOT2 inputs low and TIM4 counter/channels disabled.
- Explicit stop passed at the end of that inspection.

The diagnostic script now checks PWM modes/remapping and optionally samples
GPIO input readback for both logic levels on all six active MCU pins. This is
not a scope measurement of PWM frequency/duty or proof of driver output/current.
A subsequent attempt lost ST-LINK USB communication before reaching pin sampling
(`DEV_USB_COMM_ERR`); its final stop was not verified. Keep motor power off until
the probe connection and stop state are checked again.

After the user reconnected ST-LINK with 3S still disconnected, the full check
passed: all six active MCU pins showed both high and low input-readback levels
(PA masks `0xcc/0xcc`, PB masks `0x3/0x3`), PWM/remap checks passed, and explicit
stop passed. Firmware is now running with both commanded motor groups stopped.
No firmware change or connector reassignment was made during this investigation.
The connector photo/wiring confirmation and powered driver/motor checks remain
outstanding; these MCU results do not establish physical holding torque.
