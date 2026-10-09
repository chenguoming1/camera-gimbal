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
  echo FAIL: MOT0 is not in the expected 40-percent hold.\n
  motor_abort
end
if (htim3.Instance->CR1 & 1) == 0 || htim3.Instance->CCER != 0x1110 || htim3.Instance->SMCR != 0 || htim3.Instance->ARR != 3599 || htim3.Instance->PSC != 0
  echo FAIL: TIM3 carrier/output configuration.\n
  motor_abort
end
if htim3.Instance->CCR2 != 1800 || htim3.Instance->CCR3 != 2423 || htim3.Instance->CCR4 != 1177
  echo FAIL: Actual MOT0 compares differ from the fixed-field command.\n
  motor_abort
end
if (htim2.Instance->CR1 & 1) != 0 || (htim4.Instance->CR1 & 1) != 0 || htim2.Instance->CCER != 0 || htim4.Instance->CCER != 0
  echo FAIL: An inactive motor timer/output is enabled.\n
  motor_abort
end
if ($motor_gpioa->CRL & 0x0f00fff0) != 0x02002220 || ($motor_gpiob->CRH & 0xff) != 0x22 || ($motor_gpioa->ODR & 0x4e) != 0 || ($motor_gpiob->ODR & 0x300) != 0
  echo FAIL: MOT1/MOT2 input pins are not push-pull GPIOs driven low.\n
  motor_abort
end
if ($motor_gpioa->CRL & 0xf0000000) != 0xa0000000 || ($motor_gpiob->CRL & 0xff) != 0xaa
  echo FAIL: MOT0 pins are not configured for timer PWM.\n
  motor_abort
end
echo PASS: MOT0 TIM3 hold registers; MOT1/MOT2 timers disabled and inputs low.\n
set variable motor_test.request_stop = 1
motor_wait_report
if motor_test.state != MOTOR_STOPPED || htim3.Instance->CCR2 != 0 || htim3.Instance->CCR3 != 0 || htim3.Instance->CCR4 != 0
  echo FAIL: Motor stop did not clear the actual compare registers.\n
  motor_abort
end
echo PASS: Explicit stop; firmware left running with MOT0 stopped.\n
delete breakpoints
detach
