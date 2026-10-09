# 1 — Onboard MPU6050 test

- Project: `6storm32-test`
- Source: `6storm32-test/Core/Src/main.c`
- Hardware: connected STorM32 board; IMU wiring verified through SWD
- Previous LED-only checkpoint: Git tag `test-leds-alternating`

## Connections and configuration

On the connected board, the onboard MPU6050 uses **I2C1 at 7-bit address `0x69`**.
This was verified on 2026-10-09: with the external sensor connected, `0x68` and
`0x69` responded on I2C1. After unplugging the external sensor, only `0x69`
continued responding, with `WHO_AM_I = 0x68`. Neither address responded on I2C2.

This differs from the reference v1.30 schematic, which shows I2C2 and AD0 low
(`0x68`). The test follows the measured wiring of the connected board. Set
`IMU_I2C_HANDLE` and `MPU6050_ADDRESS_7BIT` in `main.c` to select a different
board's wiring. HAL receives the shifted address, currently `0x69 << 1`.
The device address and identity register are different: `WHO_AM_I` is still
`0x68` when the device address is `0x69`.

| Signal | MCU pin | Existing CubeMX configuration |
| --- | --- | --- |
| IMU SCL | PB6 | I2C1_SCL, alternate function open drain |
| IMU SDA | PB7 | I2C1_SDA, alternate function open drain |
| USB D− / D+ | PA11 / PA12 | USB device, CDC class |
| USB connection control | PB5 | GPIO output open drain |
| Green LED | PB12 | GPIO output push pull |
| Red LED | PB13 | GPIO output push pull |

I2C1 is already configured at 100 kHz. The
test uses polling, so no IMU interrupt pin or I²C DMA configuration is needed.
USB already has a 48 MHz clock and its interrupt enabled. Application startup
releases PB5 high after USB initialization, enabling the board's USB connection.

## What the code does

1. Reads `WHO_AM_I` at register `0x75`, requiring `0x68`.
2. Resets the sensor, waits, wakes it using the X-gyro PLL clock, and enables all
   six motion axes and the temperature sensor.
3. Sets ±2 g acceleration, ±250 degrees/second gyro range, DLPF setting 3, and
   a 50 Hz sensor sample rate. Each configuration write is read back and checked.
4. Reads 14 consecutive bytes starting at `0x3B`: acceleration X/Y/Z,
   temperature, and gyro X/Y/Z. Reading them together keeps the values coherent.
5. Converts signed raw counts into integer units and stores the result in
   `imu_test`. The main loop requests a sample approximately every 20 ms.
6. Attempts a USB report every 250 ms. A missing or busy USB connection skips
   reports without stopping sensor sampling. The transmit buffer remains valid
   until USB completes the transfer.
7. Retries sensor initialization after an error, with a one-second interval
   between attempts. Failed reads retain the last sample but mark it invalid.

Successful reads blink green with a 250 ms toggle interval; red is off. An IMU
identity, configuration, or I²C error turns red on and green off. These LEDs
indicate communication status; the motion checks below verify sensor behavior.
The earlier alternating LED test is replaced by these status indications.

## Read the output over USB

1. Build and flash `6storm32-test` from STM32CubeIDE. Resume execution if the
   debugger pauses after programming.
2. Connect the board's USB data port to your computer.
3. Open its USB CDC serial port in a terminal. On macOS, look for
   `/dev/cu.usbmodem*`. Select 115200 baud, 8 data bits, no parity, one stop bit,
   and no flow control. USB CDC carries the data; the selected baud does not
   determine this test's report rate.
4. Observe approximately four lines per second. Reports repeat so opening the
   terminal after startup still shows the current state.

Example only; actual readings depend on orientation, motion, temperature, and bias:

```text
IMU OK bus=I2C1 addr=0x69 id=0x68 n=50 accel_mg=[8,-12,1002] gyro_mdps=[152,-305,76] temp_centiC=2853 errors=0
```

| Field | Meaning |
| --- | --- |
| `bus` / `addr` | Selected I²C bus and 7-bit device address |
| `id` | Last successfully read identity; expected `0x68` |
| `n` | Cumulative successful sample count, including initialization samples |
| `accel_mg` | Sensor X/Y/Z acceleration in milligravity units; `1000` = 1 g |
| `gyro_mdps` | Sensor X/Y/Z rotation rate in millidegrees/second; `1000` = 1 degree/second |
| `temp_centiC` | Sensor die temperature in hundredths of a degree Celsius; `2853` = 28.53 °C |
| `errors` | Cumulative failed I²C transactions, identity checks, or configuration checks |

The conversion uses 16384 counts/g, 131 counts/(degree/second), and
`temperature_C = raw / 340 + 36.53`. Integer division truncates the scaled output.
Axes follow the sensor, without a gimbal frame rotation or calibration.

## Inspect the result without USB

Add `imu_test` to CubeIDE **Live Expressions**, or pause execution and inspect it
in the debugger. It contains the state, identity, raw and scaled motion readings,
temperature, sample/error counts, last HAL/I²C error, and last successful sample
time. Sampling and LEDs continue without a USB host.

Only use the retained readings while `imu_test.state == IMU_TEST_OK` (`1`).
Pausing the MCU also pauses sampling and can interrupt USB communication.

## Check the sensor physically

1. Hold the board still. The acceleration vector's magnitude should be roughly
   1000 mg. With a sensor axis vertical, that axis should read about ±1000 mg,
   while the other two are near zero. The sign depends on orientation.
2. Slowly tilt the board. Gravity should move between the acceleration axes.
3. Rotate the board about each axis. The corresponding gyro reading should
   change and reverse sign when you reverse the rotation. At rest, gyro readings
   should be near zero, with some uncalibrated bias and noise.
4. Verify that `n` keeps increasing and that `errors` stays stable, ideally zero.
   A green heartbeat alone does not prove the sensor is measuring correctly.

This is a communication and motion response test. It does not run the MPU6050
factory self-test, calibrate offsets, or calculate roll/pitch/yaw. Motor PWM
outputs are not started by this test.

## Diagnose failures

An error report looks like:

```text
IMU ERROR bus=I2C1 addr=0x69 state=2 id=0x00 hal=1 i2c=0x00000004 errors=1
```

| `state` | Meaning |
| --- | --- |
| `0` | Startup, before the first completed test |
| `1` | Identity/configuration passed and the last sample read succeeded |
| `2` | I²C transaction failed; inspect `last_hal_status` and `last_i2c_error` |
| `3` | Device answered with an unexpected `WHO_AM_I` |
| `4` | A configuration register did not read back as written |

`hal` uses HAL status values: `0` OK, `1` error, `2` busy, `3` timeout. `i2c` is
the HAL I²C error bitmask; for example, `0x04` is acknowledge failure.

- **Red stays on:** verify board power, I2C1 PB6/PB7, pull-ups, and the sensor
  address/identity. This test currently requires an MPU6050 at `0x69`.
- **Green blinks but USB is absent:** check the USB data cable, CDC port, PB5
  release, and that firmware execution is running. Inspect `imu_test` via SWD.
- **Port exists but reports stop:** close and reopen the serial terminal. The
  code skips reports while a previous USB transfer remains busy.
- **Bus stays busy or times out:** inspect SCL/SDA and power-cycle the board.
  Retries reconfigure the sensor but do not generate GPIO bus recovery pulses.
- **No LED activity and no samples:** check whether startup stopped in
  `Error_Handler()` before reaching the IMU test.

## Register reference and validation

Register meanings and scaling follow TDK's
[MPU-6000/MPU-6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
and [product specification](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet.pdf).
The local STorM32 v1.30 schematic sheets are in `storm32/`; their IMU bus/address
mapping differs from the connected board, as described above.

### Debug process — 2026-10-09

1. **Check the connection and build.** STM32CubeProgrammer detected STLINK-V3SET.
   The project built successfully with CubeIDE's bundled ARM toolchain. The only
   USB serial device detected belonged to ST-LINK, so the investigation used SWD
   and GDB rather than the board's USB CDC output.
2. **Flash and inspect startup.** Start ST-LINK GDB server in SWD mode, connect
   GDB with `Debug/6storm32-test.elf`, load/verify the firmware, reset, and stop
   after IMU initialization. Set a breakpoint in `Error_Handler()` to catch a
   peripheral initialization failure. Inspect `imu_test` and `imu_initialized`.
3. **Identify the initial failure.** The original schematic-based test used I2C2
   at `0x68`. Live status showed `IMU_TEST_I2C_ERROR`, `HAL_ERROR`, I²C error
   `0x04` (acknowledge failure), and zero successful samples. Repeated attempts
   produced the same result; the main loop was running, but the device did not
   acknowledge that address on that bus.
4. **Probe both buses and addresses.** Call `HAL_I2C_Mem_Read()` from GDB to read
   register `0x75` (`WHO_AM_I`) at `0x68` and `0x69` on each I²C handle. Pass the
   address shifted left once. Read the identity only after `HAL_OK`; a failed
   transaction leaves the destination byte unchanged. The unused
   `HAL_I2C_IsDeviceReady()` function was not linked into this firmware, so direct
   register reads were used instead.
5. **Separate onboard from external.** Initially both addresses answered on
   I2C1. The user unplugged the external IMU, and the same checks were repeated:

   | Bus | Address | External connected | External unplugged |
   | --- | --- | --- | --- |
   | I2C1 | `0x68` | `HAL_OK`, identity `0x68` | Acknowledge failure |
   | I2C1 | `0x69` | `HAL_OK`, identity `0x68` | `HAL_OK`, identity `0x68` |
   | I2C2 | `0x68` | Acknowledge failure | Acknowledge failure |
   | I2C2 | `0x69` | Acknowledge failure | Acknowledge failure |

   The device remaining at I2C1/`0x69` was identified as onboard. Before it was
   initialized, its power register read `0x40` (sleep enabled) and motion
   registers were zero. Those initial zeros did not establish a sensor fault.
6. **Correct the application.** Change `IMU_I2C_HANDLE` to `hi2c1` and
   `MPU6050_ADDRESS_7BIT` to `0x69`. Keep the expected identity at `0x68`. Include
   the selected bus/address in debugger status and USB reports so later tests
   show which sensor is being read.
7. **Rebuild, flash, and observe live status.** Verify the flash download, reset,
   and inspect `imu_test` again. Identity and configuration checks passed. Run
   between snapshots and confirm successful samples increase with no new errors.
   The measured results are recorded below. `SystemCoreClock` was 72 MHz.
8. **Finish the session.** Delete test breakpoints, detach GDB to resume the
   application, and stop the diagnostic GDB server. The IMU test remains running
   on the board.

### Replay the debug process with the scripts

The scripts are in `scripts/`:

- `debug-onboard-imu.sh` locates CubeIDE's bundled tools, selects the ST-LINK
  serial from `6storm32-test.launch` unless overridden, checks that the chosen
  debug port is unused, starts the server, and captures logs.
- `imu-debug.gdb` contains native GDB commands for the bus/address probes and
  live sampling checks. CubeIDE's bundled GDB does not support Python scripting;
  Python 3 runs on the Mac only for tool setup and ELF comparison.

End any existing CubeIDE debug session first. From the repository root, inspect
the available modes:

```bash
scripts/debug-onboard-imu.sh --help
```

To repeat the address checks, leave ST-LINK and board power connected, disconnect
the board's USB data cable, and run:

```bash
scripts/debug-onboard-imu.sh probe
```

Probe mode attaches without flashing or resetting the sensor. It first reads
flash bytes and compares `.isr_vector`, `.text`, and `.rodata` with the local ELF
before calling target functions. The ST-LINK server's CRC-based
`compare-sections` reported mismatches even when direct byte comparisons matched,
so the script uses direct memory dumps instead.

It then stops at `IMU_TestReport()`, after the application's I²C operations have
finished, and probes both addresses on both buses. The USB-data restriction
keeps the temporary diagnostic byte separate from an active CDC transmission.
The byte is restored afterward. Repeat with the external IMU connected and
unplugged to reproduce the identification table above.

To build, flash/verify, reset, and check live samples for five seconds:

```bash
scripts/debug-onboard-imu.sh verify
```

For a longer run or another ST-LINK probe:

```bash
IMU_VERIFY_SECONDS=30 scripts/debug-onboard-imu.sh verify
IMU_STLINK_SERIAL=YOUR_PROBE_SERIAL scripts/debug-onboard-imu.sh probe
```

Verify mode checks `IMU_TEST_OK`, an increasing sample count, and no increase in
the error count during the observation interval. The duration uses firmware
uptime, excluding time paused at breakpoints. Both modes delete their diagnostic
breakpoints and detach afterward. Logs remain in the temporary directory printed
by the wrapper. If startup stops in `Error_Handler()`, the script prints a
backtrace and fails. Interrupt a stalled session with Ctrl+C and inspect its logs.

Both script modes were exercised against the connected board. The five-second
`verify` run passed with successful samples increasing from `12` to `251` and
errors remaining `0`. Probe mode confirmed I2C1/`0x69` identity `0x68`, with
acknowledge failures at the other three bus/address combinations.

### Hardware debug result — 2026-10-09

The corrected firmware was flashed and verified through ST-LINK V3/SWD with the
external IMU unplugged:

- Onboard device: I2C1 at `0x69`; identity `0x68`.
- Startup state: `IMU_TEST_OK`, all configuration readbacks passed.
- Successful samples increased from `1` to `2111` over approximately 45 seconds
  of firmware uptime; the error count remained `0`.
- Final sample: acceleration `[1, 258, 933]` mg (magnitude approximately 0.97 g),
  gyro `[-4312, -1167, -251]` millidegrees/second, temperature `34.18` °C.
- The nonzero gyro readings include uncalibrated bias; no offsets were applied.

This confirms identity, configuration, and sustained live reads from the onboard
sensor. Deliberate rotation/tilt response has not yet been verified. USB CDC
reporting has not yet been verified on the host: only ST-LINK's serial port was
connected during this debug session. Complete the physical motion checks above
before treating motion response as verified.
