# Requires matching flash/ELF (checked by wrapper) and DISCONNECTED motor power.
# Inspect actual timer/GPIO registers, then leave the motor stopped.
define motor_abort
  set variable motor_test.request_stop = 1
  delete breakpoints
  detach
  quit 1
end

define motor_wait_report
  tbreak IMU_TestReport
  continue
end

hbreak Error_Handler
commands
  silent
  echo FAIL: Error_Handler reached.\n
  backtrace
  motor_abort
end

set $motor_gpioa = (GPIO_TypeDef*)0x40010800
set $motor_gpiob = (GPIO_TypeDef*)0x40010c00
motor_wait_report
if motor_test.state == MOTOR_STOPPED
  set variable motor_test.request_run = 1
end
set $motor_wait_start = uwTick
while motor_test.state != MOTOR_HOLD && motor_test.state != MOTOR_FAULT && (unsigned int)(uwTick - $motor_wait_start) < 4000
  motor_wait_report
end
print motor_test
printf "TIM2 CR1/CCER: 0x%x / 0x%x\n", htim2.Instance->CR1, htim2.Instance->CCER
printf "TIM3 CR1/CCER: 0x%x / 0x%x\n", htim3.Instance->CR1, htim3.Instance->CCER
printf "TIM4 CR1/CCER: 0x%x / 0x%x\n", htim4.Instance->CR1, htim4.Instance->CCER
printf "GPIOA CRL/ODR: 0x%x / 0x%x\n", $motor_gpioa->CRL, $motor_gpioa->ODR
printf "GPIOB CRL/CRH/ODR: 0x%x / 0x%x / 0x%x\n", $motor_gpiob->CRL, $motor_gpiob->CRH, $motor_gpiob->ODR
if motor_test.state != MOTOR_HOLD || motor_test.drive_permille != 400
  echo FAIL: MOT0/MOT1/MOT2 are not in the expected 40-percent hold.\n
  motor_abort
end
if (htim3.Instance->CR1 & 1) == 0 || htim3.Instance->CCER != 0x1111 || htim3.Instance->SMCR != 0x16 || htim3.Instance->ARR != 3599 || htim3.Instance->PSC != 0
  echo FAIL: TIM3 carrier/output configuration.\n
  motor_abort
end
if htim3.Instance->CCR2 != 1800 || htim3.Instance->CCR3 != 2423 || htim3.Instance->CCR4 != 1177
  echo FAIL: Actual MOT0 compares differ from the fixed-field command.\n
  motor_abort
end
if (htim2.Instance->CR1 & 1) == 0 || htim2.Instance->CCER != 0x1110 || (htim2.Instance->CR2 & 0x70) != 0x10 || htim2.Instance->SMCR != 0 || htim2.Instance->ARR != 3599 || htim2.Instance->PSC != 0
  echo FAIL: TIM2 carrier/output/master configuration.\n
  motor_abort
end
if htim3.Instance->CCR1 != 1800 || htim2.Instance->CCR4 != 2423 || htim2.Instance->CCR3 != 1177
  echo FAIL: Actual MOT1 compares differ from the fixed-field command.\n
  motor_abort
end
if (htim4.Instance->CR1 & 1) == 0 || htim4.Instance->CCER != 0x1100 || htim4.Instance->SMCR != 0x16 || htim4.Instance->ARR != 3599 || htim4.Instance->PSC != 0
  echo FAIL: TIM4 carrier/output/slave configuration.\n
  motor_abort
end
if htim4.Instance->CCR4 != 1800 || htim2.Instance->CCR2 != 2423 || htim4.Instance->CCR3 != 1177
  echo FAIL: Actual MOT2 compares differ from the fixed-field command.\n
  motor_abort
end
if ($motor_gpioa->CRL & 0xff00fff0) != 0xaa00aaa0 || ($motor_gpiob->CRH & 0xff) != 0xaa
  echo FAIL: Motor pins are not configured for timer PWM.\n
  motor_abort
end
if ($motor_gpioa->CRL & 0xf0000000) != 0xa0000000 || ($motor_gpiob->CRL & 0xff) != 0xaa
  echo FAIL: MOT0 pins are not configured for timer PWM.\n
  motor_abort
end
# CEN/CCR alone do not establish PWM mode or default pin routing.
printf "TIM3 CCMR1/CCMR2: 0x%x / 0x%x; TIM2 CCMR2: 0x%x\n", htim3.Instance->CCMR1, htim3.Instance->CCMR2, htim2.Instance->CCMR2
set $motor_afio = (AFIO_TypeDef*)0x40010000
printf "AFIO MAPR: 0x%x\n", $motor_afio->MAPR
if (htim3.Instance->CCMR1 & 0xffff) != 0x6868 || (htim3.Instance->CCMR2 & 0xffff) != 0x6868 || (htim2.Instance->CCMR2 & 0xffff) != 0x6868 || (htim4.Instance->CCMR2 & 0xffff) != 0x6868 || (htim2.Instance->CCMR1 & 0xff00) != 0x6800 || ($motor_afio->MAPR & 0x1f00) != 0
  echo FAIL: PWM mode/preload or TIM2/TIM3/TIM4 pin remapping differs from expected.\n
  motor_abort
end
# Sample physical GPIO input readback while autonomous timers run. This
# confirms both logic levels were observed, not waveform frequency/duty/torque.
set $motor_debug_cr = *(unsigned int*)0xe0042004
printf "DBGMCU CR: 0x%x\n", $motor_debug_cr
if ($motor_debug_cr & 0x700) == 0
  set $motor_high_a = 0
  set $motor_low_a = 0
  set $motor_high_b = 0
  set $motor_low_b = 0
  set $motor_sample = 0
  while $motor_sample < 64
    set $motor_a = $motor_gpioa->IDR
    set $motor_b = $motor_gpiob->IDR
    set $motor_high_a = $motor_high_a | ($motor_a & 0xce)
    set $motor_low_a = $motor_low_a | ((~$motor_a) & 0xce)
    set $motor_high_b = $motor_high_b | ($motor_b & 0x303)
    set $motor_low_b = $motor_low_b | ((~$motor_b) & 0x303)
    set $motor_sample = $motor_sample + 1
  end
  printf "GPIO input readback high/low masks: PA=0x%x/0x%x (expect 0xce/0xce); PB=0x%x/0x%x (expect 0x303/0x303)\n", $motor_high_a, $motor_low_a, $motor_high_b, $motor_low_b
  if $motor_high_a == 0xce && $motor_low_a == 0xce && $motor_high_b == 0x303 && $motor_low_b == 0x303
    echo PASS: Both logic levels observed on all nine active motor MCU pins.\n
  else
    echo WARNING: Some active motor pins did not show both levels; use a scope to distinguish a stuck output from sampling aliasing.\n
  end
else
  echo SKIP: Pin-level sampling because debug freeze stops an active timer.\n
end
echo PASS: MOT0/MOT1/MOT2 TIM2/TIM3/TIM4 hold registers.\n
set variable motor_test.request_stop = 1
motor_wait_report
if htim4.Instance->CCR4 != 0 || htim2.Instance->CCR2 != 0 || htim4.Instance->CCR3 != 0 || htim3.Instance->CCR1 != 0 || htim2.Instance->CCR3 != 0 || htim2.Instance->CCR4 != 0 || motor_test.state != MOTOR_STOPPED || htim3.Instance->CCR2 != 0 || htim3.Instance->CCR3 != 0 || htim3.Instance->CCR4 != 0
  echo FAIL: Motor stop did not clear the actual compare registers.\n
  motor_abort
end
echo PASS: Explicit stop; firmware left running with MOT0/MOT1/MOT2 stopped.\n
delete breakpoints
detach
