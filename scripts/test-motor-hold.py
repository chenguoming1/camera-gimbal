#!/usr/bin/env python3
"""Compile actual MOT0 hold functions with HAL mocks; never access hardware."""
from pathlib import Path
import os
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[1] / '6storm32-test/Core/Src/main.c').read_text()
def between(start, end):
    return source.split(start, 1)[1].split(end, 1)[0]
mock = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
enum { HAL_OK, HAL_ERROR, GPIO_PIN_RESET, GPIO_PIN_SET,
       GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW,
       GPIOA, GPIOB, GPIOC, TIM_CHANNEL_2=2, TIM_CHANNEL_3=3, TIM_CHANNEL_4=4 };
#define GPIO_PIN_1 (1U << 1)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_3 (1U << 3)
#define GPIO_PIN_6 (1U << 6)
#define GPIO_PIN_8 (1U << 8)
#define GPIO_PIN_9 (1U << 9)
#define TIM_CR1_CEN 1U
#define TIM_CCER_CC2E 0x10U
#define TIM_CCER_CC3E 0x100U
#define TIM_CCER_CC4E 0x1000U
#define TIM_EGR_UG 1U
#define IMU_TEST_OK 1
static struct {int state;} imu_test;
typedef struct {uint32_t CR1, SMCR, CCER, PSC, ARR, EGR, CCR[5];} Timer;
typedef struct {Timer *Instance;} TIM_HandleTypeDef;
typedef struct {uint32_t Pin, Mode, Pull, Speed;} GPIO_InitTypeDef;
static Timer tim2, tim3, tim4;
static TIM_HandleTypeDef htim2={&tim2}, htim3={&tim3}, htim4={&tim4};
static uint32_t now, gpio_low[16], gpio_outputs[16];
static int pressed, fail_start, starts;
#define __HAL_TIM_SET_COMPARE(h,ch,v) ((h)->Instance->CCR[ch]=(v))
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->Instance->ARR)
static uint32_t HAL_GetTick(void) {return now;}
static int HAL_GPIO_ReadPin(int port, uint32_t pin) {
  assert(port==GPIOC && pin==GPIO_PIN_3); return pressed ? GPIO_PIN_RESET : GPIO_PIN_SET;
}
static void HAL_GPIO_WritePin(int port, uint32_t pins, int state) {
  assert(state==GPIO_PIN_RESET); gpio_low[port]|=pins;
}
static void HAL_GPIO_Init(int port, GPIO_InitTypeDef *init) {
  assert(init->Mode==GPIO_MODE_OUTPUT_PP && init->Pull==GPIO_NOPULL);
  assert((gpio_low[port]&init->Pin)==init->Pin); gpio_outputs[port]|=init->Pin;
}
static int HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, int ch) {
  assert(timer==&htim3 && ch>=2 && ch<=4); starts++;
  if(starts==fail_start) return HAL_ERROR;
  timer->Instance->CCER |= 1U << ((ch-1)*4);
  timer->Instance->CR1 |= TIM_CR1_CEN;
  return HAL_OK;
}
'''
firmware = (
    between('/* Fixed-field component', '/* USER CODE END PTD */').split('*/', 1)[1]
    + '\n#define MOTOR_HOLD_DRIVE_PERMILLE'
    + between('#define MOTOR_HOLD_DRIVE_PERMILLE', '/* USER CODE END PD */')
    + '\nvolatile Motor_TestStatus motor_test;\n'
    + between('volatile Motor_TestStatus motor_test;', '/* USER CODE END PV */')
    + '\n' + between('/* USER CODE BEGIN 0 */', '/* Record failed HAL transactions')
)
checks = r'''
static void zero(void) {
  assert(motor_test.drive_permille==0 && tim3.CCR[2]==0 && tim3.CCR[3]==0 && tim3.CCR[4]==0);
  assert(tim3.EGR==TIM_EGR_UG);
}
static void reset(void) {
  memset((void*)&motor_test,0,sizeof(motor_test));
  memset(&tim2,0,sizeof(tim2)); memset(&tim3,0,sizeof(tim3)); memset(&tim4,0,sizeof(tim4));
  memset(gpio_low,0,sizeof(gpio_low)); memset(gpio_outputs,0,sizeof(gpio_outputs));
  tim3.ARR=3599; now=0; pressed=starts=fail_start=motor_initialized=0;
}
static void tick(void) {
  now++; Motor_TestTick();
  assert(tim2.CR1==0 && tim4.CR1==0 && tim2.CCER==0 && tim4.CCER==0);
  assert(tim3.CCER==0x1110U || motor_test.state==MOTOR_FAULT);
  assert(tim3.CCR[1]==0); /* MOT1 PA6 channel never starts. */
  assert(motor_test.drive_permille<=400);
}
static void ticks(int n) {while(n--)tick();}
int main(void) {
  reset(); tim2.CR1=tim4.CR1=1; tim2.CCER=tim4.CCER=0x1111; tim3.SMCR=6;
  Motor_TestInit(); assert(starts==3 && tim3.SMCR==0); zero();
  assert(gpio_outputs[GPIOA]==(GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6));
  assert(gpio_outputs[GPIOB]==(GPIO_PIN_8|GPIO_PIN_9));
  imu_test.state=2; /* Motor hold independent of IMU success. */
  ticks(1999); zero(); assert(motor_test.runs==0);
  tick(); assert(motor_test.state==MOTOR_RAMP && motor_test.runs==1);
  ticks(500); assert(motor_test.drive_permille==200);
  ticks(500); assert(motor_test.state==MOTOR_HOLD && motor_test.drive_permille==400);
  assert(tim3.CCR[2]==1800 && tim3.CCR[3]==2423 && tim3.CCR[4]==1177);
  ticks(30000); assert(motor_test.state==MOTOR_HOLD && motor_test.runs==1);
  now=motor_started_ms-1U; tick(); assert(motor_test.state==MOTOR_HOLD && tim3.CCR[3]==2423);
  pressed=1; tick(); zero(); assert(motor_test.stop_reason==MOTOR_STOP_BUTTON);
  pressed=0; ticks(5000); zero(); assert(motor_test.runs==1);
  motor_test.request_run=1; tick(); ticks(1000); assert(motor_test.state==MOTOR_HOLD && motor_test.runs==2);
  motor_test.request_stop=1; tick(); zero(); assert(motor_test.stop_reason==MOTOR_STOP_COMMAND);
  reset(); pressed=1; Motor_TestInit(); ticks(4000); zero(); assert(motor_test.runs==0);
  reset(); Motor_TestInit(); motor_test.request_stop=1; tick(); ticks(4000); zero(); assert(motor_test.runs==0);
  reset(); now=UINT32_MAX-100; Motor_TestInit(); ticks(3000); assert(motor_test.state==MOTOR_HOLD);
  for(int i=1;i<=3;i++) {reset(); fail_start=i; Motor_TestInit(); zero(); assert(motor_test.state==MOTOR_FAULT);}
  reset(); tim3.ARR=999; Motor_TestInit(); zero(); assert(motor_test.state==MOTOR_FAULT && starts==0);
  reset(); Motor_TestInit(); ticks(3000); tim3.CR1=0; tick(); zero(); assert(motor_test.stop_reason==MOTOR_STOP_TIMER);
  reset(); Motor_TestInit(); ticks(3000); tim3.CCER &= ~TIM_CCER_CC3E; tick(); zero(); assert(motor_test.state==MOTOR_FAULT);
  reset(); Motor_TestInit(); ticks(3000); Motor_TestEmergencyStop(); zero();
  assert(motor_test.state==MOTOR_FAULT && motor_test.stop_reason==MOTOR_STOP_FAULT);
  motor_test.request_run=1; ticks(5000); zero(); assert(motor_test.runs==1);
  puts("PASS: MOT0-only pins, inactive motor GPIOs/timers, 40% ramp/hold, stops, timer/fault interlocks, rollover, IMU independence");
}
'''
with tempfile.TemporaryDirectory(prefix='motor-hold-logic-') as temp:
    cfile=Path(temp)/'test.c'; executable=Path(temp)/'test'
    cfile.write_text(mock+firmware+checks)
    subprocess.run([os.environ.get('CC','clang'),'-std=c11','-Wall','-Wextra','-Werror',str(cfile),'-o',str(executable)],check=True)
    subprocess.run([str(executable)],check=True,timeout=10)
