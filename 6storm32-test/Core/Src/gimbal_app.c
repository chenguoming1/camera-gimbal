#include "main.h"
#include "gimbal_app.h"
#include "gimbal_motor.h"
#include "gimbal_config.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
extern I2C_HandleTypeDef hi2c1;
extern USBD_HandleTypeDef hUsbDeviceFS;
volatile Gimbal_Status gimbal;
static Gimbal_Attitude attitude;
static Gimbal_PID pid[3];
static float raw_a[3], raw_w[3], mean[6], m2[6], smoothed[3], demand[3];
static uint8_t sensor_ready, calibrated, level_captured;
static volatile uint8_t initialized;
static int level_z;
static uint32_t next_sample, retry_ms, report_ms, arm_ms, last_cycles;
static volatile uint16_t rx_head, rx_tail;
static uint8_t rx[256];
static char line[128], tx[768];
static unsigned line_length;
static uint8_t discard_line;
static volatile uint8_t rx_poison;
static const char *state_name[]={"WAIT_IMU","CALIBRATING","DISARMED","RAMP","ACTIVE","FAULT"};
static void reply(const char *fmt, ...)
{
  char buf[128]; va_list ap; va_start(ap,fmt); vsnprintf(buf,sizeof(buf),fmt,ap); va_end(ap);
  for(unsigned i=0;i<sizeof(buf);i++) { gimbal.reply[i]=buf[i]; if(!buf[i]) break; }
}
static void fault(Gimbal_Fault reason)
{
  uint32_t irq=__get_PRIMASK(); __disable_irq();
  Gimbal_MotorStop();
  if(gimbal.state!=GIMBAL_FAULT) gimbal.fault=reason;
  gimbal.state=GIMBAL_FAULT;
  __set_PRIMASK(irq);
}
void Gimbal_EmergencyStop(void) { fault(GIMBAL_CPU_FAULT); }
static void stop(void)
{
  uint32_t irq=__get_PRIMASK(); __disable_irq();
  Gimbal_MotorStop();
  if(gimbal.state!=GIMBAL_FAULT) gimbal.state=sensor_ready?(calibrated?GIMBAL_DISARMED:GIMBAL_CALIBRATING):GIMBAL_WAIT_IMU;
  rx_tail=rx_head; line_length=0;
  __set_PRIMASK(irq);
}
void Gimbal_Tick(void)
{
  if(!initialized) return;
  if(gimbal.request_stop || !(GPIOC->IDR&GPIO_PIN_3)) {
    gimbal.request_stop=0;
    fault((GPIOC->IDR&GPIO_PIN_3)?GIMBAL_STOP:GIMBAL_BUTTON);
  }
  if(gimbal.state==GIMBAL_ACTIVE || gimbal.state==GIMBAL_RAMP) {
    if((uint32_t)(HAL_GetTick()-gimbal.last_sample_ms)>25) fault(GIMBAL_STALE);
    else if(!Gimbal_MotorHealthy()) fault(GIMBAL_TIMER);
  }
}
void Gimbal_Receive(const uint8_t *data, uint32_t length)
{
  /* Single USB IRQ producer. Ctrl-C and overflow stop before main-loop parsing. */
  for(uint32_t i=0;i<length;i++) {
    if(data[i]==3) { fault(GIMBAL_STOP); rx_poison=1; return; }
    uint16_t next=(rx_head+1U)&255U;
    if(next==rx_tail) { fault(GIMBAL_RX_OVERFLOW); rx_poison=1; return; }
    rx[rx_head]=data[i]; __DMB(); rx_head=next;
  }
}
static int reg_read(uint8_t reg, uint8_t *value, uint16_t length)
{
  HAL_StatusTypeDef s=HAL_I2C_Mem_Read(&hi2c1,0x68U<<1,reg,I2C_MEMADD_SIZE_8BIT,value,length,4);
  gimbal.last_i2c_error=HAL_I2C_GetError(&hi2c1);
  return s==HAL_OK;
}
static int reg_write(uint8_t reg, uint8_t value)
{
  uint8_t actual;
  return HAL_I2C_Mem_Write(&hi2c1,0x68U<<1,reg,I2C_MEMADD_SIZE_8BIT,&value,1,4)==HAL_OK
    && reg_read(reg,&actual,1) && actual==value;
}
static int bus_recover(void)
{
  /* Resetting during a transfer can leave the still-powered MPU holding SDA.
   * Recovery runs only with motor drive stopped: open-drain clocks, then STOP.
   * HAL_Init restores AF_OD and resets the F1 I2C peripheral/analog filter.
   */
  if(HAL_I2C_DeInit(&hi2c1)!=HAL_OK) return 0;
  GPIO_InitTypeDef pins={0};
  pins.Pin=GPIO_PIN_6|GPIO_PIN_7; pins.Mode=GPIO_MODE_OUTPUT_OD;
  pins.Pull=GPIO_NOPULL; pins.Speed=GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_WritePin(GPIOB,pins.Pin,GPIO_PIN_SET);HAL_GPIO_Init(GPIOB,&pins);
  for(int i=0;i<9;i++) {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);HAL_Delay(1);
    if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)!=GPIO_PIN_SET) { (void)HAL_I2C_Init(&hi2c1);return 0; }
  }
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);HAL_Delay(1);
  int released=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)==GPIO_PIN_SET;
  return HAL_I2C_Init(&hi2c1)==HAL_OK && released;
}
static int imu_init(void)
{
  uint8_t id=0, reset=0x80;
  gimbal.address=0x68;
  if(!bus_recover()) return 0;
  if(!reg_read(0x75,&id,1)) return 0;
  gimbal.who_am_i=id;
  if(id!=0x68) return 0;
  if(HAL_I2C_Mem_Write(&hi2c1,0x68U<<1,0x6B,1,&reset,1,4)!=HAL_OK) return 0;
  HAL_Delay(100);
  if(!reg_write(0x6B,1)) return 0;
  HAL_Delay(100);
  /* 1 kHz filtered sample clock / (4+1) = 200 Hz; +/-500 dps, +/-4 g. */
  return reg_write(0x6C,0) && reg_write(0x19,4) && reg_write(0x1A,3)
    && reg_write(0x1B,8) && reg_write(0x1C,8) && reg_write(0x38,1);
}
static int16_t signed_word(const uint8_t *p) { return (int16_t)(((uint16_t)p[0]<<8)|p[1]); }
static int imu_read(void)
{
  uint8_t b[15];
  if(!reg_read(0x3A,b,sizeof(b))) { gimbal.errors++; fault(GIMBAL_IMU_ERROR); sensor_ready=0; return -1; }
  if(!(b[0]&1)) return 0; /* No fresh DATA_RDY; never integrate an old sample. */
  for(int i=0;i<3;i++) {
    gimbal.accel_raw[i]=signed_word(&b[1+2*i]);
    gimbal.gyro_raw[i]=signed_word(&b[9+2*i]);
    raw_a[i]=gimbal.accel_raw[i]/8192.0f;
    raw_w[i]=gimbal.gyro_raw[i]/65.5f;
    smoothed[i]+=0.04f*(raw_a[i]-smoothed[i]);
  }
  gimbal.samples++; gimbal.last_sample_ms=HAL_GetTick();
  return 1;
}
static void calibration_start(void)
{
  uint32_t irq=__get_PRIMASK(); __disable_irq();
  if(gimbal.state==GIMBAL_FAULT) { __set_PRIMASK(irq); return; }
  Gimbal_MotorStop(); calibrated=0; gimbal.calibration_samples=0;
  memset(mean,0,sizeof(mean)); memset(m2,0,sizeof(m2));
  for(int i=0;i<3;i++) gimbal.bias_dps[i]=0;
  gimbal.state=GIMBAL_CALIBRATING;
  __set_PRIMASK(irq);
}
static void calibration_step(void)
{
  float norm=0;
  for(int i=0;i<3;i++) norm+=raw_a[i]*raw_a[i];
  if(norm<0.81f||norm>1.21f||fabsf(raw_w[0])>5||fabsf(raw_w[1])>5||fabsf(raw_w[2])>5) {
    calibration_start(); return;
  }
  unsigned n=++gimbal.calibration_samples;
  for(int i=0;i<6;i++) {
    float x=i<3?raw_w[i]:raw_a[i-3];
    float delta=x-mean[i]; mean[i]+=delta/n; m2[i]+=delta*(x-mean[i]);
  }
  if(n<400) return;
  for(int i=0;i<6;i++) if(m2[i]/(n-1)>(i<3?0.25f:0.0004f)) { calibration_start(); return; }
  int axes[3]; float a[3];
  for(int i=0;i<3;i++) { gimbal.bias_dps[i]=mean[i]; axes[i]=gimbal.axes[i]; }
  Gimbal_Map(raw_a,axes,a); Gimbal_AttitudeInit(&attitude,a);
  uint32_t irq=__get_PRIMASK(); __disable_irq();
  if(gimbal.state==GIMBAL_CALIBRATING) {
    calibrated=1; gimbal.state=GIMBAL_DISARMED;
    for(int i=0;i<3;i++) gimbal.target[i]=0;
  }
  __set_PRIMASK(irq);
  reply("Calibration complete; confirm axes/directions, then arm. Motors are off.");
}
static int disarmed(void) { return gimbal.state==GIMBAL_DISARMED || gimbal.state==GIMBAL_CALIBRATING; }
static int number(const char *s, float *v)
{
  if(!s||!*s) return 0;
  char *end; *v=strtof(s,&end); return *end==0 && isfinite(*v);
}
static int integer(const char *s, int *v)
{
  float f; if(!number(s,&f)||f<-1000||f>1000||f!=truncf(f)) return 0; *v=(int)f; return 1;
}
static void axes_apply(const int axes[3])
{
  for(int i=0;i<3;i++) gimbal.axes[i]=axes[i];
  gimbal.orientation_ok=1; level_captured=0; calibration_start();
  reply("Axes=%d,%d,%d; keep camera still for calibration.",axes[0],axes[1],axes[2]);
}
static int orientation_capture(int noseup)
{
  if(!calibrated||!disarmed()) return 0;
  float norm=0, speed=0;
  for(int i=0;i<3;i++) { norm+=smoothed[i]*smoothed[i]; speed+=fabsf(raw_w[i]-gimbal.bias_dps[i]); }
  if(norm<0.9f||norm>1.1f||speed>2) return 0;
  if(!noseup) {
    int z=0; for(int i=1;i<3;i++) if(fabsf(smoothed[i])>fabsf(smoothed[z])) z=i;
    if(fabsf(smoothed[z])<0.97f) return 0;
    level_z=(smoothed[z]>0?-1:1)*(z+1); level_captured=1;
    reply("Level captured. Tilt lens UP 30-45 degrees, hold still, send noseup."); return 1;
  }
  if(!level_captured) return 0;
  int z=abs(level_z)-1,x=(z+1)%3;
  for(int i=0;i<3;i++) if(i!=z && fabsf(smoothed[i])>fabsf(smoothed[x])) x=i;
  int y=3-x-z;
  float za=(level_z>0?1:-1)*smoothed[z];
  if(za>-.5f||za<-.94f||fabsf(smoothed[x])<.34f||fabsf(smoothed[y])>.15f) return 0;
  int a[3]={(smoothed[x]>0?1:-1)*(x+1),y+1,level_z};
  if(!Gimbal_ValidAxes(a)) a[1]=-a[1];
  axes_apply(a); return 1;
}
static void command(char *cmd)
{
  char *v[8], *save=NULL; int n=0;
  for(char *p=strtok_r(cmd," \t",&save);p && n<8;p=strtok_r(NULL," \t",&save)) v[n++]=p;
  if(!n) return;
  gimbal.command_count++;
  if(n==1&&!strcmp(v[0],"stop")) { stop(); reply("Stopped. Motors off."); return; }
  if(n==1&&!strcmp(v[0],"status")) { reply("Status requested."); return; }
  if(n==1&&!strcmp(v[0],"help")) { reply("Commands: status stop calibrate level noseup axes dir enable power gains target hold center arm"); return; }
  if(n==1&&!strcmp(v[0],"calibrate") && gimbal.state!=GIMBAL_ACTIVE && gimbal.state!=GIMBAL_RAMP) {
    if(!(GPIOC->IDR&GPIO_PIN_3)) goto bad;
    sensor_ready=0; Gimbal_MotorStop();
    if(!Gimbal_MotorHealthy()) goto bad;
    uint32_t irq=__get_PRIMASK(); __disable_irq();
    gimbal.fault=GIMBAL_OK; gimbal.state=GIMBAL_CALIBRATING;
    __set_PRIMASK(irq);
    calibration_start();
    sensor_ready=imu_init();
    if(!sensor_ready) { gimbal.errors++; fault(GIMBAL_IMU_ERROR); reply("External IMU 0x68 initialization failed."); }
    next_sample=HAL_GetTick(); last_cycles=DWT->CYCCNT; return;
  }
  if(n==1&&(!strcmp(v[0],"level")||!strcmp(v[0],"noseup"))) {
    if(orientation_capture(!strcmp(v[0],"noseup"))) return;
    goto bad;
  }
  if(n==4&&!strcmp(v[0],"axes")&&disarmed()) {
    int a[3]; for(int i=0;i<3;i++) if(!integer(v[i+1],&a[i])) goto bad;
    if(!Gimbal_ValidAxes(a)) goto bad;
    axes_apply(a); return;
  }
  if(n==4&&(!strcmp(v[0],"dir")||!strcmp(v[0],"enable")||!strcmp(v[0],"power"))&&disarmed()) {
    int a[3]; for(int i=0;i<3;i++) if(!integer(v[i+1],&a[i])) goto bad;
    if(!strcmp(v[0],"dir")) {
      for(int i=0;i<3;i++) if(a[i]!=1&&a[i]!=-1) goto bad;
      for(int i=0;i<3;i++) gimbal.direction[i]=a[i];
      gimbal.direction_ok=1;
    } else if(!strcmp(v[0],"enable")) {
      for(int i=0;i<3;i++) if(a[i]<0||a[i]>1) goto bad;
      for(int i=0;i<3;i++) gimbal.enabled[i]=a[i];
    } else {
      for(int i=0;i<3;i++) if(a[i]<0||a[i]>400) goto bad;
      for(int i=0;i<3;i++) gimbal.power[i]=(uint16_t)a[i];
    }
    if(!strcmp(v[0],"dir"))
      reply("Direction signs roll=%d pitch=%d yaw=%d; selection unchanged.",a[0],a[1],a[2]);
    else if(!strcmp(v[0],"enable"))
      reply("Selected roll(MOT2)=%d pitch(MOT1)=%d yaw(MOT0)=%d; arm required.",a[0],a[1],a[2]);
    else
      reply("Power set; configuration is RAM only.");
    return;
  }
  if(n==5&&!strcmp(v[0],"gains")&&disarmed()) {
    int axis; float a[3]; if(!integer(v[1],&axis)||axis<0||axis>2) goto bad;
    for(int i=0;i<3;i++) if(!number(v[i+2],&a[i])||a[i]<0||a[i]>100) goto bad;
    pid[axis].kp=a[0];pid[axis].ki=a[1];pid[axis].kd=a[2]; reply("Gains accepted."); return;
  }
  if(n==4&&!strcmp(v[0],"target")&&calibrated) {
    float a[3]; for(int i=0;i<3;i++) if(!number(v[i+1],&a[i])) goto bad;
    if(fabsf(a[0])>30||fabsf(a[1])>45||fabsf(a[2])>180) goto bad;
    for(int i=0;i<3;i++) gimbal.target[i]=a[i];
    reply("Target accepted; slew limits apply."); return;
  }
  if(n==1&&(!strcmp(v[0],"hold")||!strcmp(v[0],"center"))&&calibrated) {
    for(int i=0;i<3;i++) gimbal.target[i]=!strcmp(v[0],"hold")?attitude.angle[i]:(i==2?attitude.angle[2]:0);
    reply("Target captured."); return;
  }
  if(n==1&&!strcmp(v[0],"arm")) {
    int selected=0;
    for(int i=0;i<3;i++) selected|=gimbal.enabled[i] && gimbal.power[i]>0;
    if(!selected) {
      gimbal.command_errors++;
      reply("Arm rejected: select a motor first, e.g. enable 1 0 0.");
      return;
    }
    if(gimbal.state!=GIMBAL_DISARMED||!calibrated||!sensor_ready||!gimbal.orientation_ok||!gimbal.direction_ok
       ||gimbal.fault!=GIMBAL_OK||(uint32_t)(HAL_GetTick()-gimbal.last_sample_ms)>15
       ||fabsf(attitude.angle[0])>60||fabsf(attitude.angle[1])>60) goto bad;
    for(int i=0;i<3;i++) { pid[i].integral=0; pid[i].phase=0; demand[i]=attitude.angle[i]; }
    uint32_t irq=__get_PRIMASK(); __disable_irq();
    int armed_ok=Gimbal_MotorArm();
    if(armed_ok) { arm_ms=HAL_GetTick(); gimbal.state=GIMBAL_RAMP; }
    __set_PRIMASK(irq);
    if(!armed_ok) { fault(GIMBAL_TIMER); goto bad; }
    reply("Armed; one-second alignment ramp."); return;
  }
bad:
  gimbal.command_errors++; reply("Rejected: invalid command, state, range, or unconfirmed axes/directions.");
}
static void commands(void)
{
  /* Bound parsing work per task so USB traffic cannot starve the 200 Hz loop. */
  for(int budget=0;budget<64;budget++) {
    uint32_t irq=__get_PRIMASK(); __disable_irq();
    if(rx_poison) { rx_tail=rx_head; rx_poison=0; line_length=0; discard_line=1; }
    if(rx_tail==rx_head) { __set_PRIMASK(irq); break; }
    uint8_t c=rx[rx_tail]; rx_tail=(rx_tail+1U)&255U;
    __set_PRIMASK(irq);
    if(c=='\r'||c=='\n') {
      if(!discard_line) { line[line_length]=0; command(line); }
      line_length=0; discard_line=0;
    } else if(!discard_line) {
      if(c<32||c>126||line_length>=sizeof(line)-1) {
        discard_line=1;line_length=0; gimbal.command_errors++; reply("Rejected: invalid or oversized line.");
      } else line[line_length++]=(char)c;
    }
  }
}
void Gimbal_Report(void)
{
  uint32_t irq=__get_PRIMASK(); __disable_irq();
  USBD_CDC_HandleTypeDef *cdc=hUsbDeviceFS.pClassData;
  int ready=hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED&&cdc&&cdc->TxState==0;
  __set_PRIMASK(irq); if(!ready) return;
  /* Integer telemetry avoids requiring printf-float in CubeIDE's nano library. */
  int len=snprintf(tx,sizeof(tx),
    "state=%s fault=%u order=roll,pitch,yaw motor_ports=2,1,0 imu=0x%02X id=0x%02X samples=%lu errors=%lu cal=%lu "
    "axes=%d,%d,%d orient=%u dir=%d,%d,%d dir_ok=%u enable=%d,%d,%d power=%u,%u,%u "
    "angle_mdeg=%ld,%ld,%ld target_mdeg=%ld,%ld,%ld accel_mg=%ld,%ld,%ld gyro_mdps=%ld,%ld,%ld "
    "loop_us=%lu max_us=%lu commands=%lu rejected=%lu reply=%s\r\n",
    state_name[gimbal.state],(unsigned)gimbal.fault,gimbal.address,gimbal.who_am_i,
    (unsigned long)gimbal.samples,(unsigned long)gimbal.errors,(unsigned long)gimbal.calibration_samples,
    gimbal.axes[0],gimbal.axes[1],gimbal.axes[2],gimbal.orientation_ok,
    gimbal.direction[0],gimbal.direction[1],gimbal.direction[2],gimbal.direction_ok,
    gimbal.enabled[0],gimbal.enabled[1],gimbal.enabled[2],gimbal.power[0],gimbal.power[1],gimbal.power[2],
    (long)(1000*gimbal.angle[0]),(long)(1000*gimbal.angle[1]),(long)(1000*gimbal.angle[2]),
    (long)(1000*gimbal.target[0]),(long)(1000*gimbal.target[1]),(long)(1000*gimbal.target[2]),
    (long)(1000*gimbal.accel_g[0]),(long)(1000*gimbal.accel_g[1]),(long)(1000*gimbal.accel_g[2]),
    (long)(1000*gimbal.gyro_dps[0]),(long)(1000*gimbal.gyro_dps[1]),(long)(1000*gimbal.gyro_dps[2]),
    (unsigned long)gimbal.control_us,(unsigned long)gimbal.max_control_us,
    (unsigned long)gimbal.command_count,(unsigned long)gimbal.command_errors,(const char*)gimbal.reply);
  if(len<=0||(unsigned)len>=sizeof(tx)) return;
  irq=__get_PRIMASK(); __disable_irq(); cdc=hUsbDeviceFS.pClassData;
  if(hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED&&cdc&&cdc->TxState==0) (void)CDC_Transmit_FS((uint8_t*)tx,(uint16_t)len);
  __set_PRIMASK(irq);
}
void Gimbal_Init(void)
{
  const int axes[3]=GIMBAL_DEFAULT_AXES, directions[3]=GIMBAL_DEFAULT_DIRECTIONS;
  const int enabled[3]=GIMBAL_DEFAULT_ENABLED;
  const uint16_t power[3]=GIMBAL_DEFAULT_POWER;
  for(int i=0;i<3;i++) {
    gimbal.axes[i]=axes[i];gimbal.direction[i]=directions[i];gimbal.enabled[i]=enabled[i];gimbal.power[i]=power[i];
    pid[i].kp=GIMBAL_DEFAULT_KP;pid[i].ki=GIMBAL_DEFAULT_KI;pid[i].kd=GIMBAL_DEFAULT_KD;
  }
  if(!Gimbal_ValidAxes(axes)) { fault(GIMBAL_TIMING); initialized=1; return; }
  gimbal.orientation_ok=GIMBAL_ORIENTATION_CONFIGURED;
  gimbal.direction_ok=GIMBAL_DIRECTIONS_CONFIGURED;
  CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk; DWT->CYCCNT=0;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
  if(!Gimbal_MotorInit()) { fault(GIMBAL_TIMER); initialized=1; return; }
  sensor_ready=imu_init();
  if(sensor_ready) calibration_start();
  else { gimbal.errors++;gimbal.state=GIMBAL_WAIT_IMU;reply("Waiting for external MPU6050 at I2C1/0x68."); }
  next_sample=retry_ms=HAL_GetTick(); last_cycles=DWT->CYCCNT; initialized=1;
  /* Independent watchdog: /32, reload=624, about 0.5 s at nominal 40 kHz LSI.
   * Freeze only while SWD halts the CPU; reset always boots DISARMED again.
   */
  DBGMCU->CR|=DBGMCU_CR_DBG_IWDG_STOP;
  IWDG->KR=0x5555; IWDG->PR=3; IWDG->RLR=624;
  uint32_t start=HAL_GetTick();
  while(IWDG->SR && (uint32_t)(HAL_GetTick()-start)<10) {}
  IWDG->KR=0xAAAA;IWDG->KR=0xCCCC;
}
void Gimbal_Task(void)
{
  IWDG->KR=0xAAAA;
  uint32_t now=HAL_GetTick();
  if(gimbal.state==GIMBAL_WAIT_IMU && (uint32_t)(now-retry_ms)>=1000) {
    sensor_ready=imu_init(); retry_ms=next_sample=HAL_GetTick(); last_cycles=DWT->CYCCNT;
    if(sensor_ready) calibration_start(); else gimbal.errors++;
    now=HAL_GetTick();
  }
  if(sensor_ready && (int32_t)(now-next_sample)>=0) {
    next_sample=now+5;
    uint32_t cycle=DWT->CYCCNT, elapsed=cycle-last_cycles;
    int fresh=imu_read();
    if(fresh>0) {
      last_cycles=cycle;
      int axes[3];float a[3],w[3],corrected[3];
      for(int i=0;i<3;i++) { axes[i]=gimbal.axes[i];corrected[i]=raw_w[i]-gimbal.bias_dps[i]; }
      Gimbal_Map(raw_a,axes,a);Gimbal_Map(corrected,axes,w);
      for(int i=0;i<3;i++) { gimbal.accel_g[i]=a[i];gimbal.gyro_dps[i]=w[i]; }
      if(gimbal.state==GIMBAL_CALIBRATING) calibration_step();
      else if(calibrated && gimbal.state!=GIMBAL_FAULT) {
        float dt=elapsed/(float)SystemCoreClock;
        if(!Gimbal_AttitudeUpdate(&attitude,a,w,dt)) fault(GIMBAL_TIMING);
        else for(int i=0;i<3;i++) gimbal.angle[i]=attitude.angle[i];
        if(gimbal.state==GIMBAL_ACTIVE||gimbal.state==GIMBAL_RAMP) {
          if(fabsf(attitude.angle[0])>60||fabsf(attitude.angle[1])>60) fault(GIMBAL_TILT);
          else {
            float phase[3];uint16_t power[3];
            uint32_t ramp=HAL_GetTick()-arm_ms;
            for(int i=0;i<3;i++) {
              if(ramp>=1000) {
                demand[i]=Gimbal_Wrap(demand[i]+Gimbal_Clamp(Gimbal_Wrap(gimbal.target[i]-demand[i]),-(i==2?20:10)*dt,(i==2?20:10)*dt));
                if(gimbal.enabled[i]) Gimbal_PIDStep(&pid[i],Gimbal_Wrap(demand[i]-attitude.angle[i]),attitude.rate[i],dt,gimbal.direction[i]);
              }
              phase[i]=pid[i].phase;gimbal.phase[i]=phase[i];
              power[i]=gimbal.enabled[i]?(uint16_t)(gimbal.power[i]*(ramp<1000?ramp:1000)/1000):0;
            }
            uint32_t irq=__get_PRIMASK(); __disable_irq();
            if(ramp>=1000 && gimbal.state==GIMBAL_RAMP) gimbal.state=GIMBAL_ACTIVE;
            __set_PRIMASK(irq);
            if(!Gimbal_MotorApply(phase,power) && gimbal.state!=GIMBAL_FAULT) fault(GIMBAL_TIMER);
          }
        }
      }
    } else if(fresh==0 && (uint32_t)(HAL_GetTick()-gimbal.last_sample_ms)>25) fault(GIMBAL_STALE);
    gimbal.control_us=(uint32_t)(DWT->CYCCNT-cycle)/(SystemCoreClock/1000000U);
    if(gimbal.control_us>gimbal.max_control_us) gimbal.max_control_us=gimbal.control_us;
  }
  commands();
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,(gimbal.state==GIMBAL_FAULT||gimbal.state==GIMBAL_WAIT_IMU)?GPIO_PIN_SET:GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,gimbal.state==GIMBAL_ACTIVE?GPIO_PIN_SET:((HAL_GetTick()/250)&1?GPIO_PIN_SET:GPIO_PIN_RESET));
  if((uint32_t)(HAL_GetTick()-report_ms)>=250) { report_ms=HAL_GetTick();Gimbal_Report(); }
}
