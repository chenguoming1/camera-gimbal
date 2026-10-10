#!/usr/bin/env python3
"""Compile the real application, motor driver and filter with host HAL mocks.
No ST-LINK, serial connection or hardware access. Requires clang (or CC).
"""
from pathlib import Path
import os
import subprocess
import tempfile

repo = Path(__file__).resolve().parents[1]
core = repo / '6storm32-test/Core'
mock = r'''
#ifndef MOCK_MAIN_H
#define MOCK_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef struct { uint32_t CR1,CR2,SMCR,CCER,PSC,ARR,EGR,CNT,CCR1,CCR2,CCR3,CCR4; } Timer;
typedef struct { Timer *Instance; } TIM_HandleTypeDef;
typedef struct { uint32_t IDR; } GPIO;
typedef struct { uint32_t Pin,Mode,Pull,Speed; } GPIO_InitTypeDef;
typedef struct { uint32_t ErrorCode; } I2C_HandleTypeDef;
typedef int HAL_StatusTypeDef;
typedef struct { uint32_t CYCCNT,CTRL; } DW;
typedef struct { uint32_t DEMCR; } CD;
typedef struct { uint32_t KR,PR,RLR,SR; } IW;
typedef struct { uint32_t CR; } DBG;
extern Timer tim2,tim3,tim4;
extern GPIO ga,gb,gc;
extern DW dw;
extern CD cd;
extern IW iw;
extern DBG dbg;
#define TIM2 (&tim2)
#define TIM3 (&tim3)
#define TIM4 (&tim4)
#define GPIOA (&ga)
#define GPIOB (&gb)
#define GPIOC (&gc)
#define DWT (&dw)
#define CoreDebug (&cd)
#define IWDG (&iw)
#define DBGMCU (&dbg)
#define CoreDebug_DEMCR_TRCENA_Msk 1
#define DWT_CTRL_CYCCNTENA_Msk 1
#define DBGMCU_CR_DBG_IWDG_STOP 0x100
#define GPIO_PIN_0 1U
#define GPIO_PIN_1 2U
#define GPIO_PIN_2 4U
#define GPIO_PIN_3 8U
#define GPIO_PIN_6 64U
#define GPIO_PIN_7 128U
#define GPIO_PIN_8 256U
#define GPIO_PIN_9 512U
#define GPIO_PIN_12 4096U
#define GPIO_PIN_13 8192U
#define GPIO_MODE_AF_PP 2
#define GPIO_MODE_OUTPUT_OD 3
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_LOW 0
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define TIM_CHANNEL_1 1
#define TIM_CHANNEL_2 2
#define TIM_CHANNEL_3 3
#define TIM_CHANNEL_4 4
#define TIM_CR1_CEN 1U
#define TIM_CR2_MMS 0x70U
#define TIM_TRGO_ENABLE 0x10U
#define TIM_TS_ITR1 0x10U
#define TIM_SLAVEMODE_TRIGGER 6U
#define TIM_EGR_UG 1U
#define I2C_MEMADD_SIZE_8BIT 1
#define HAL_OK 0
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->Instance->ARR)
#define __DMB() ((void)0)
extern uint32_t SystemCoreClock;
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
void HAL_GPIO_Init(GPIO *port, GPIO_InitTypeDef *p);
void HAL_GPIO_WritePin(GPIO *p, uint32_t pin, int state);
int HAL_GPIO_ReadPin(GPIO *p,uint32_t pin);
int HAL_I2C_Init(I2C_HandleTypeDef *h);
int HAL_I2C_DeInit(I2C_HandleTypeDef *h);
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *h,int ch);
int HAL_I2C_Mem_Read(I2C_HandleTypeDef *h,int address,int reg,int size,uint8_t *data,int len,int timeout);
int HAL_I2C_Mem_Write(I2C_HandleTypeDef *h,int address,int reg,int size,uint8_t *data,int len,int timeout);
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *h);
#endif
'''
usb = r'''
#ifndef MOCK_USB_H
#define MOCK_USB_H
#include <stdint.h>
typedef struct { uint32_t TxState; } USBD_CDC_HandleTypeDef;
typedef struct { int dev_state; void *pClassData; } USBD_HandleTypeDef;
#define USBD_STATE_CONFIGURED 3
int CDC_Transmit_FS(uint8_t *data,uint16_t len);
#endif
'''
harness = r'''
#include "main.h"
#include "usbd_cdc_if.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
Timer tim2,tim3,tim4;
GPIO ga,gb,gc;
DW dw; CD cd; IW iw; DBG dbg;
uint32_t SystemCoreClock=72000000;
TIM_HandleTypeDef htim2={&tim2},htim3={&tim3},htim4={&tim4};
I2C_HandleTypeDef hi2c1;
USBD_HandleTypeDef hUsbDeviceFS;
static uint32_t now,irq;
static int fail_i2c, stale, start_fail, starts;
static uint8_t regs[128];
static uint32_t af_a,af_b;
static int16_t accel[3]={0,0,-8192},gyro[3];
uint32_t __get_PRIMASK(void) { return irq; }
void __disable_irq(void) { irq=1; }
void __set_PRIMASK(uint32_t mask) { irq=mask; }
static void trigger(void) { if(tim2.CR1&1) { if(tim3.SMCR==0x16) tim3.CR1|=1; if(tim4.SMCR==0x16) tim4.CR1|=1; } }
uint32_t HAL_GetTick(void) { trigger();return now; }
void HAL_Delay(uint32_t ms) { now+=ms;dw.CYCCNT+=ms*72000; }
void HAL_GPIO_Init(GPIO *p,GPIO_InitTypeDef *init) { if(init->Mode==GPIO_MODE_AF_PP) {if(p==GPIOA) af_a|=init->Pin;else af_b|=init->Pin;} else assert(init->Mode==GPIO_MODE_OUTPUT_OD); }
void HAL_GPIO_WritePin(GPIO *p,uint32_t pin,int state) { if(state) p->IDR|=pin;else p->IDR&=~pin; }
int HAL_GPIO_ReadPin(GPIO *p,uint32_t pin) {return (p->IDR&pin)?GPIO_PIN_SET:GPIO_PIN_RESET;}
int HAL_I2C_Init(I2C_HandleTypeDef *h) {(void)h;return 0;}
int HAL_I2C_DeInit(I2C_HandleTypeDef *h) {(void)h;return 0;}
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *h,int ch) { if(++starts==start_fail) return 1;h->Instance->CCER|=1U<<((ch-1)*4);if((h->Instance->SMCR&7)!=6) h->Instance->CR1|=1;trigger();return 0; }
int HAL_I2C_Mem_Read(I2C_HandleTypeDef *h,int address,int reg,int size,uint8_t *b,int len,int timeout) {
  assert(h==&hi2c1 && address==0xD0 && size==1 && timeout==4);dw.CYCCNT+=72000;
  if(fail_i2c) {h->ErrorCode=4;return 1;} h->ErrorCode=0;
  if(reg==0x3A) {assert(len==15);memset(b,0,15);b[0]=stale?0:1;for(int i=0;i<3;i++){b[1+2*i]=(uint16_t)accel[i]>>8;b[2+2*i]=accel[i]&255;b[9+2*i]=(uint16_t)gyro[i]>>8;b[10+2*i]=gyro[i]&255;}}
  else {assert(len==1);b[0]=regs[reg];}return 0;
}
int HAL_I2C_Mem_Write(I2C_HandleTypeDef *h,int address,int reg,int size,uint8_t *b,int len,int timeout) {
  assert(h==&hi2c1 && address==0xD0 && size==1 && len==1 && timeout==4);if(fail_i2c)return 1;regs[reg]=*b;return 0;
}
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *h) { return h->ErrorCode; }
int CDC_Transmit_FS(uint8_t *data,uint16_t len) { assert(len<768 && strstr((char*)data,"state="));return 0; }
#include "gimbal_app.c"
#include "gimbal_motor.c"
static void zero(void) { assert(!tim2.CCR2&&!tim2.CCR3&&!tim2.CCR4&&!tim3.CCR1&&!tim3.CCR2&&!tim3.CCR3&&!tim3.CCR4&&!tim4.CCR3&&!tim4.CCR4); }
static void step(unsigned count) { while(count--) {now+=5;dw.CYCCNT+=360000;Gimbal_Tick();Gimbal_Task();} }
static void send(const char *s) { Gimbal_Receive((const uint8_t*)s,(uint32_t)strlen(s));while(rx_tail!=rx_head) commands(); }
static void setup(void) {
  memset((void*)&gimbal,0,sizeof(gimbal));memset(&tim2,0,sizeof(tim2));memset(&tim3,0,sizeof(tim3));memset(&tim4,0,sizeof(tim4));
  memset(mean,0,sizeof(mean));memset(m2,0,sizeof(m2));memset(smoothed,0,sizeof(smoothed));memset(regs,0,sizeof(regs));
  regs[0x75]=0x68;tim2.ARR=tim3.ARR=tim4.ARR=3599;gc.IDR=8;now=dw.CYCCNT=irq=0;
  initialized=calibrated=sensor_ready=armed=level_captured=0;line_length=discard_line=rx_head=rx_tail=rx_poison=0;
  starts=start_fail=fail_i2c=stale=0;report_ms=0;gyro[0]=gyro[1]=gyro[2]=0;accel[0]=accel[1]=0;accel[2]=-8192;
}
static void ready(void) { setup();Gimbal_Init();step(401);assert(gimbal.state==GIMBAL_DISARMED);zero(); }
static void arm(void) { send("axes 1 2 3\n");step(401);send("enable 1 0 0\npower 400 400 400\ndir 1 1 1\narm\n");assert(gimbal.state==GIMBAL_RAMP);step(220);assert(gimbal.state==GIMBAL_ACTIVE); }
int main(void) {
  setbuf(stdout,NULL); puts("filter");
  /* Gravity tilt, gyro yaw integration, normalization and rejected reflections. */
  int axes[3]={1,-2,-3},bad[3]={1,2,-3};assert(Gimbal_ValidAxes(axes)&&!Gimbal_ValidAxes(bad));
  Gimbal_Attitude s;float a[3]={.5f,0,-.8660254f},w[3]={0};Gimbal_AttitudeInit(&s,a);assert(fabsf(s.angle[1]-30)<.01f);
  a[0]=0;a[2]=-1;Gimbal_AttitudeInit(&s,a);w[2]=90;for(int i=0;i<200;i++)assert(Gimbal_AttitudeUpdate(&s,a,w,.005f));assert(fabsf(s.angle[2]-90)<.01f);
  w[2]=0;Gimbal_AttitudeInit(&s,(float[3]){.5f,0,-.8660254f});for(int i=0;i<1000;i++)assert(Gimbal_AttitudeUpdate(&s,a,w,.005f));assert(fabsf(s.angle[1])<.01f);
  assert(!Gimbal_AttitudeUpdate(&s,a,w,.1f));w[0]=NAN;assert(!Gimbal_AttitudeUpdate(&s,a,w,.005f));
  Gimbal_PID p={35,5,8,0,0};for(int i=0;i<10000;i++)Gimbal_PIDStep(&p,90,0,.005f,1);assert(fabsf(p.integral)<.01f&&fabsf(p.phase)<=180);
  puts("startup defaults"); ready();assert(gimbal.address==0x68 && gimbal.who_am_i==0x68 && gimbal.orientation_ok && gimbal.direction_ok);
  assert(gimbal.axes[0]==-2 && gimbal.axes[1]==1 && gimbal.axes[2]==3);
  assert(gimbal.direction[0]==1 && gimbal.direction[1]==-1 && gimbal.direction[2]==-1);
  for(int i=0;i<3;i++) { assert(gimbal.enabled[i]==1 && gimbal.power[i]==200);assert(pid[i].kp==5 && pid[i].ki==0 && pid[i].kd==8); }
  assert(af_a==(2|4|8|64|128) && af_b==(1|2|256|512));step(1000);zero();assert(gimbal.state==GIMBAL_DISARMED);
  /* Tested defaults permit an explicit all-axis arm; never start automatically. */
  send("hold\narm\n");step(220);assert(gimbal.state==GIMBAL_ACTIVE);
  assert(tim3.CCR2 && tim3.CCR3 && tim3.CCR4 && tim3.CCR1 && tim2.CCR4 && tim2.CCR3 && tim4.CCR4 && tim2.CCR2 && tim4.CCR3);
  send("stop\n");zero();
  send("enable 0 0 0\narm\n");assert(gimbal.state==GIMBAL_DISARMED);zero();
  assert(strstr((const char*)gimbal.reply,"select a motor first"));
  send("enable 1 0 0\n");gimbal.orientation_ok=0;send("arm\n");assert(gimbal.state==GIMBAL_DISARMED);zero();
  gimbal.orientation_ok=1;gimbal.direction_ok=0;send("arm\n");assert(gimbal.state==GIMBAL_DISARMED);zero();
  puts("orientation");
  /* Discover the commissioned mapping: body X=-raw Y, body Y=raw X. */
  smoothed[0]=smoothed[1]=0;smoothed[2]=-1;send("level\n");assert(level_captured);
  smoothed[0]=0;smoothed[1]=-.5f;smoothed[2]=-.8660254f;send("noseup\n");assert(gimbal.orientation_ok && gimbal.axes[0]==-2 && gimbal.axes[1]==1 && gimbal.axes[2]==3);
  puts("parser"); ready();send("axes 1 2 -3\n");assert(gimbal.axes[0]==-2 && gimbal.axes[1]==1 && gimbal.orientation_ok);send("power 401 400 400\ntarget nan 0 0\ngains 0 inf 1 1\n");assert(gimbal.command_errors==4);
  /* Fragmented CDC packets and complete numeric validation. */
  send("ax");send("es 1 2 3\r\n");assert(gimbal.orientation_ok);step(401);gimbal.direction_ok=0;send("dir 1 1 1junk\narm\n");assert(gimbal.state==GIMBAL_DISARMED);
  puts("arm"); arm();assert(!tim3.CCR2&&!tim3.CCR3&&!tim3.CCR4&&!tim2.CCR3&&!tim2.CCR4&&!tim3.CCR1);assert(tim4.CCR4==1800 && tim2.CCR2==2423 && tim4.CCR3==1176); // independent float truncation
  float phases[3]={25,-45,80};uint16_t power[3]={100,200,300},c[3];assert(Gimbal_MotorApply(phases,power));
  Gimbal_PWM(25,100,c);assert(tim4.CCR4==c[0]&&tim2.CCR2==c[1]&&tim4.CCR3==c[2]);
  Gimbal_PWM(-45,200,c);assert(tim3.CCR1==c[0]&&tim2.CCR4==c[1]&&tim2.CCR3==c[2]);
  Gimbal_PWM(80,300,c);assert(tim3.CCR2==c[0]&&tim3.CCR3==c[1]&&tim3.CCR4==c[2]);
  Gimbal_MotorStop();assert(!Gimbal_MotorApply(phases,power));zero();
  /* Actual application selection must drive exactly one connector per axis. */
  ready();arm();send("stop\n");send("enable 0 1 0\nhold\narm\n");step(220);
  assert(gimbal.state==GIMBAL_ACTIVE && tim3.CCR1 && tim2.CCR4 && tim2.CCR3);
  assert(!tim3.CCR2&&!tim3.CCR3&&!tim3.CCR4&&!tim4.CCR4&&!tim2.CCR2&&!tim4.CCR3);
  send("stop\n");send("enable 0 0 1\nhold\narm\n");step(220);
  assert(gimbal.state==GIMBAL_ACTIVE && tim3.CCR2 && tim3.CCR3 && tim3.CCR4);
  assert(!tim3.CCR1&&!tim2.CCR4&&!tim2.CCR3&&!tim4.CCR4&&!tim2.CCR2&&!tim4.CCR3);
  ready();arm();send("stop\narm\n");zero();assert(gimbal.state==GIMBAL_DISARMED); // flush queued arm
  ready();arm();gc.IDR=0;Gimbal_Tick();zero();assert(gimbal.fault==GIMBAL_BUTTON);gc.IDR=8;step(100);zero();send("arm\n");assert(gimbal.state==GIMBAL_FAULT);
  ready();arm();fail_i2c=1;step(1);zero();assert(gimbal.fault==GIMBAL_IMU_ERROR);fail_i2c=0;step(100);zero();assert(gimbal.state==GIMBAL_FAULT);
  send("calibrate\n");step(401);assert(gimbal.state==GIMBAL_DISARMED);zero();
  ready();arm();stale=1;step(6);zero();assert(gimbal.fault==GIMBAL_STALE);
  ready();arm();now+=26;Gimbal_Tick();zero();assert(gimbal.fault==GIMBAL_STALE);
  ready();arm();tim4.SMCR=0;Gimbal_Tick();zero();assert(gimbal.fault==GIMBAL_TIMER);
  ready();arm();Gimbal_Receive((uint8_t*)"\003",1);zero();assert(gimbal.fault==GIMBAL_STOP);
  ready();arm();char big[300];memset(big,'a',sizeof(big));Gimbal_Receive((uint8_t*)big,sizeof(big));zero();assert(gimbal.fault==GIMBAL_RX_OVERFLOW);
  ready();char longline[210];memset(longline,' ',200);memcpy(longline+200,"arm\n",5);send(longline);assert(gimbal.state==GIMBAL_DISARMED);
  setup();gyro[0]=1000;Gimbal_Init();step(500);assert(gimbal.state==GIMBAL_CALIBRATING);zero();
  setup();fail_i2c=1;Gimbal_Init();assert(gimbal.state==GIMBAL_WAIT_IMU);step(300);zero();assert(gimbal.state==GIMBAL_WAIT_IMU);
  for(int i=1;i<=9;i++){setup();start_fail=i;Gimbal_Init();assert(gimbal.state==GIMBAL_FAULT);zero();}
  ready();arm();gimbal.request_stop=1;Gimbal_Tick();zero();assert(gimbal.state==GIMBAL_FAULT);
  puts("PASS: fusion, axes capture, PID limits, calibration, USB parsing, arm gates, 9-phase mapping, stop races, sensor/stale/timer/button faults");
}
'''
with tempfile.TemporaryDirectory(prefix='gimbal-host-') as directory:
    temp = Path(directory)
    (temp/'main.h').write_text(mock)
    (temp/'usbd_cdc_if.h').write_text(usb)
    (temp/'test.c').write_text(harness)
    executable = temp/'test'
    subprocess.run([os.environ.get('CC','clang'),'-std=c11','-D_POSIX_C_SOURCE=200809L','-Wall','-Wextra','-Werror','-g', '-I'+str(temp),'-I'+str(core/'Inc'),'-I'+str(core/'Src'),str(temp/'test.c'),str(core/'Src/gimbal_control.c'),'-lm','-o',str(executable)],check=True)
    subprocess.run([str(executable)],check=True,timeout=30)
