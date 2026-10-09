# Invoked by debug-onboard-imu.sh after connecting to ST-LINK.
# Uses native GDB commands: CubeIDE's bundled GDB has no Python support.

define imu_abort
  delete breakpoints
  detach
  quit 1
end

define imu_wait_report
  # Stop after I2C operations finish; avoid reentering a locked HAL handle.
  tbreak IMU_TestReport
  continue
end

define imu_check_status
  if imu_test.state != IMU_TEST_OK
    echo FAIL: IMU is not healthy.\n
    print imu_test
    imu_abort
  end
end

define imu_probe_address
  # Arguments: bus number, I2C handle pointer, 7-bit address.
  set variable imu_usb_line[0] = 0x55
  set $imu_hal = (int)HAL_I2C_Mem_Read($arg1, ($arg2 << 1), 0x75, 1, (unsigned char*)imu_usb_line, 1, 50)
  printf "I2C%d  0x%02X     %d           0x%02X       ", $arg0, $arg2, $imu_hal, ($arg1)->ErrorCode
  if $imu_hal == 0
    printf "0x%02X\n", (unsigned char)imu_usb_line[0]
  else
    echo --\n
  end
end

# Fail promptly if peripheral startup stops instead of reaching a report.
hbreak Error_Handler
commands
  silent
  echo FAIL: startup stopped in Error_Handler.\n
  backtrace
  imu_abort
end

if $imu_probe_mode
  # Current matching firmware is required; see the wrapper's preflight check.
  imu_wait_report
  if hUsbDeviceFS.dev_state == 3
    echo Unplug board USB data before probing; keep ST-LINK and power connected.\n
    imu_abort
  end
  set $imu_saved_byte = (unsigned char)imu_usb_line[0]
  echo \nBus   Address  HAL status  I2C error  WHO_AM_I\n
  imu_probe_address 1 &hi2c1 0x68
  imu_probe_address 1 &hi2c1 0x69
  imu_probe_address 2 &hi2c2 0x68
  imu_probe_address 2 &hi2c2 0x69
  set variable imu_usb_line[0] = $imu_saved_byte
  echo HAL 0 = success; HAL 1 / I2C 0x04 = acknowledge failure.\n
  echo Repeat with and without the external IMU to identify which address remains.\n
else
  load
  monitor reset
  imu_wait_report
  echo \nFirst live snapshot\n
  print imu_test
  imu_check_status
  set $imu_first_samples = imu_test.samples
  set $imu_first_errors = imu_test.errors
  set $imu_start_tick = uwTick
  # Measure firmware uptime; ticks do not advance while a breakpoint is held.
  while (unsigned int)(uwTick - $imu_start_tick) < ($imu_verify_seconds * 1000)
    imu_wait_report
    imu_check_status
  end
  echo \nFinal live snapshot\n
  print imu_test
  if imu_test.samples <= $imu_first_samples || imu_test.errors != $imu_first_errors
    echo FAIL: samples did not increase cleanly.\n
    imu_abort
  end
  printf "PASS: samples %u -> %u; errors %u -> %u.\n", $imu_first_samples, imu_test.samples, $imu_first_errors, imu_test.errors
  echo Motion response and host USB output need separate checks.\n
end

delete breakpoints
detach
