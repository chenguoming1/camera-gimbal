# 0 — Alternating onboard LED test

- Reference Git tag: `test-leds-alternating`
- Project: `6storm32-test`
- Source: `6storm32-test/Core/Src/main.c`
- Board: STorM32 BGC v1.30, STM32F103RCT6

## Purpose

Test the two onboard LEDs by alternating green and red continuously. This checks
the LED outputs and gives a visible indication that the firmware reaches its main
loop and that the HAL delay time base is running.

## Pins and CubeMX configuration

| LED | STM32 pin | GPIO mode | Pull | Speed | Initial output |
| --- | --- | --- | --- | --- | --- |
| Green | PB12 | Output Push Pull | No pull | Low | Low |
| Red | PB13 | Output Push Pull | No pull | Low | Low |

Both LEDs are **active high**: writing `GPIO_PIN_SET` turns the selected LED on;
writing `GPIO_PIN_RESET` turns it off. `MX_GPIO_Init()` already enables the GPIOB
clock, sets the initial outputs low, and configures these pins. No timer PWM
configuration is required for this test.

## Code

The pin definitions are in `USER CODE BEGIN PD`:

```c
#define LED_GREEN_PIN GPIO_PIN_12
#define LED_RED_PIN GPIO_PIN_13
#define LED_PORT GPIOB
#define LED_TEST_INTERVAL_MS 500U
```

The following sequence runs inside `while (1)`, in `USER CODE BEGIN 3`:

```c
/* First half of the cycle: green on, red off. */
HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
HAL_Delay(LED_TEST_INTERVAL_MS);

/* Second half of the cycle: green off, red on. */
HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);
HAL_Delay(LED_TEST_INTERVAL_MS);
```

`HAL_GPIO_WritePin()` changes an output level immediately. `HAL_Delay(500U)`
waits for 500 milliseconds using the HAL tick, which this project updates through
SysTick interrupts. Each LED stays on for approximately 500 ms, giving a complete
green/red cycle of approximately one second.

Change `LED_TEST_INTERVAL_MS` to adjust the timing of both phases. For example,
`1000U` makes each LED stay on for one second. Keep custom code inside the existing
`USER CODE` blocks so CubeMX preserves it during code regeneration.

## Run the test

1. Connect the programmer and power the board.
2. Select the `6storm32-test` project in STM32CubeIDE and build it.
3. Program and start the firmware with **Run**. If using **Debug**, press
   **Resume** if execution is paused after programming.
4. Observe green for 500 ms, followed by red for 500 ms, repeating continuously.

The test passes when both LEDs light in sequence at the expected rate. The
firmware build has passed; the physical LED behavior still needs verification on
the board.

## If the LEDs do not alternate

- **Neither LED lights:** check board power, confirm the correct firmware was
  programmed, and use a debugger breakpoint in the LED loop to confirm execution
  reaches it. An initialization failure can stop execution in `Error_Handler()`.
- **Only one LED lights:** check that PB12 and PB13 are both GPIO outputs and that
  the board matches the v1.30 pin mapping. Inspect the affected LED circuit.
- **Execution stops at a delay:** check that SysTick interrupts are running and
  that execution has been resumed in the debugger.

## Timing limitation

`HAL_Delay()` blocks the main loop during each wait, while enabled interrupts can
still run. This is sufficient for the LED component test. When adding gimbal
control or other work that must run continuously in the main loop, replace the
delays with a nonblocking sequence based on elapsed `HAL_GetTick()` time.
