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
