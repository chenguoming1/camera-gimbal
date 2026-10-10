# Native GDB only: the bundled CubeIDE GDB has no Python support.
# Invoked only with motor supply disconnected.
define gimbal_abort
  call (void)Gimbal_EmergencyStop()
  delete breakpoints
  detach
  quit 1
end

define gimbal_wait
  tbreak Gimbal_Report
  continue
end

define gimbal_zero
  if htim2.Instance->CCR2 || htim2.Instance->CCR3 || htim2.Instance->CCR4 || htim3.Instance->CCR1 || htim3.Instance->CCR2 || htim3.Instance->CCR3 || htim3.Instance->CCR4 || htim4.Instance->CCR3 || htim4.Instance->CCR4
    echo FAIL: a phase compare is nonzero while disarmed.\n
    gimbal_abort
  end
end

define gimbal_check
  if gimbal.state == GIMBAL_FAULT || gimbal.state == GIMBAL_WAIT_IMU || gimbal.address != 0x68 || gimbal.who_am_i != 0x68
    echo FAIL: external camera IMU is not healthy.\n
    print gimbal
    gimbal_abort
  end
  if gimbal.state != GIMBAL_CALIBRATING && gimbal.state != GIMBAL_DISARMED
    echo FAIL: default startup must stay disarmed.\n
    gimbal_abort
  end
  gimbal_zero
end

define gimbal_probe_address
  set variable tx[0] = 0x55
  set $result = (int)HAL_I2C_Mem_Read($arg1, ($arg2 << 1), 0x75, 1, (unsigned char*)tx, 1, 4)
  printf "I2C%d 0x%02X HAL=%d error=0x%02X", $arg0, $arg2, $result, ($arg1)->ErrorCode
  if $result == 0
    printf " WHO_AM_I=0x%02X", (unsigned char)tx[0]
  end
  echo \n
end

hbreak Error_Handler
commands
  silent
  echo FAIL: Error_Handler.\n
  backtrace
  gimbal_abort
end
if $gimbal_mode == 0
  load
  monitor reset
else
  # Request stop before resuming an existing matching image.
  set variable gimbal.request_stop = 1
end
gimbal_wait
if $gimbal_mode == 1
  if hUsbDeviceFS.dev_state == 3
    echo Disconnect board USB data before injected I2C calls; ST-LINK stays connected.\n
    gimbal_abort
  end
  gimbal_probe_address 1 &hi2c1 0x68
  gimbal_probe_address 1 &hi2c1 0x69
  gimbal_probe_address 2 &hi2c2 0x68
  gimbal_probe_address 2 &hi2c2 0x69
else
  if $gimbal_mode == 0
    gimbal_check
    set $samples = gimbal.samples
    set $errors = gimbal.errors
    set $tick = uwTick
    while (unsigned int)(uwTick - $tick) < ($imu_verify_seconds * 1000)
      gimbal_wait
      gimbal_check
    end
    if gimbal.samples <= $samples || gimbal.errors != $errors
      echo FAIL: external samples did not advance cleanly.\n
      gimbal_abort
    end
    printf "PASS: external samples %u -> %u, errors=%u; startup stayed disarmed.\n", $samples, gimbal.samples, gimbal.errors
    if gimbal.enabled[0] != 1 || gimbal.enabled[1] != 1 || gimbal.enabled[2] != 1 || gimbal.orientation_ok != 1 || gimbal.direction_ok != 1
      echo FAIL: commissioned selection/orientation/direction defaults differ.\n
      gimbal_abort
    end
    if gimbal.axes[0] != -2 || gimbal.axes[1] != 1 || gimbal.axes[2] != 3 || gimbal.direction[0] != 1 || gimbal.direction[1] != -1 || gimbal.direction[2] != -1
      echo FAIL: commissioned axes or motor signs differ.\n
      gimbal_abort
    end
    if gimbal.power[0] != 200 || gimbal.power[1] != 200 || gimbal.power[2] != 200
      echo FAIL: commissioned motor powers differ.\n
      gimbal_abort
    end
    if pid[0].kp != 5 || pid[1].kp != 5 || pid[2].kp != 5 || pid[0].ki != 0 || pid[1].ki != 0 || pid[2].ki != 0 || pid[0].kd != 8 || pid[1].kd != 8 || pid[2].kd != 8
      echo FAIL: commissioned gains differ.\n
      gimbal_abort
    end
    # Motor power MUST be disconnected. Exercise driver routing with distinct
    # strengths while main-loop feedback remains disarmed. Final reset restores
    # defaults; orientation/direction gates are bypassed only for this SWD test.
    set variable gimbal.phase[0] = 0
    set variable gimbal.phase[1] = 0
    set variable gimbal.phase[2] = 0
    set variable gimbal.power[0] = 100
    set variable gimbal.power[1] = 200
    set variable gimbal.power[2] = 300
    set $motor_arm = (int)Gimbal_MotorArm()
    set $motor_apply = (int)Gimbal_MotorApply((float*)gimbal.phase, (unsigned short*)gimbal.power)
    if $motor_arm != 1 || $motor_apply != 1
      echo FAIL: motor routing bench call failed.\n
      gimbal_abort
    end
    # MOT2=roll, strength 100; MOT1=pitch, strength 200; MOT0=yaw, strength 300.
    if htim4.Instance->CCR4 != 1800 || htim2.Instance->CCR2 != 1955 || htim4.Instance->CCR3 != 1644
      echo FAIL: roll did not route to MOT2.\n
      gimbal_abort
    end
    if htim3.Instance->CCR1 != 1800 || htim2.Instance->CCR4 != 2111 || htim2.Instance->CCR3 != 1488
      echo FAIL: pitch did not route to MOT1.\n
      gimbal_abort
    end
    if htim3.Instance->CCR2 != 1800 || htim3.Instance->CCR3 != 2267 || htim3.Instance->CCR4 != 1332
      echo FAIL: yaw did not route to MOT0.\n
      gimbal_abort
    end
    echo PASS: register routing roll=MOT2, pitch=MOT1, yaw=MOT0.\n
    set variable gimbal.request_stop = 1
    gimbal_wait
    gimbal_zero
    echo PASS: ISR stop cleared all nine nonzero motor compares.\n
    end
  print gimbal
  print /x htim2.Instance->CCER
  print /x htim3.Instance->CCER
  print /x htim4.Instance->CCER
  print /x htim3.Instance->SMCR
  print /x htim4.Instance->SMCR
  gimbal_zero
  # Confirm the request-stop ISR path latches and clears every output.
  set variable gimbal.request_stop = 1
  gimbal_wait
  if gimbal.state != GIMBAL_FAULT || gimbal.fault != GIMBAL_STOP
    echo FAIL: stop request did not latch.\n
    gimbal_abort
  end
  gimbal_zero
  echo PASS: stop request latched; all nine compares are zero.\n
end
# A final reset boots without rearming and clears the intentionally injected stop.
monitor reset
gimbal_wait
gimbal_zero
delete breakpoints
detach
