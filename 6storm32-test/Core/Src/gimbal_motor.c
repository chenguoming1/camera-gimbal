#include "main.h"
#include "gimbal_app.h"
#include "gimbal_motor.h"
#include <math.h>
extern TIM_HandleTypeDef htim2, htim3, htim4;
static volatile uint8_t armed;
void Gimbal_MotorStop(void)
{
  uint32_t irq=__get_PRIMASK(); __disable_irq();
  armed=0;
  TIM2->CCR2=TIM2->CCR3=TIM2->CCR4=0;
  TIM3->CCR1=TIM3->CCR2=TIM3->CCR3=TIM3->CCR4=0;
  TIM4->CCR3=TIM4->CCR4=0;
  TIM2->EGR=TIM_EGR_UG; TIM3->EGR=TIM_EGR_UG; TIM4->EGR=TIM_EGR_UG;
  for(int i=0;i<3;i++) for(int j=0;j<3;j++) gimbal.compare[i][j]=0;
  __set_PRIMASK(irq);
}
int Gimbal_MotorInit(void)
{
  GPIO_InitTypeDef pwm = {0};

  /* Disable all channels before restoring all nine motor pins to PWM. */
  htim3.Instance->CR1 &= ~TIM_CR1_CEN;
  htim2.Instance->CR1 &= ~TIM_CR1_CEN;
  htim4.Instance->CR1 &= ~TIM_CR1_CEN;
  htim2.Instance->CCER = 0U;
  htim4.Instance->CCER = 0U;
  htim3.Instance->CCER = 0U;
  htim2.Instance->SMCR = 0U;
  htim2.Instance->CR2 &= ~TIM_CR2_MMS;
  htim3.Instance->SMCR = 0U;
  htim4.Instance->SMCR = 0U;
  pwm.Mode = GPIO_MODE_AF_PP;
  pwm.Pull = GPIO_NOPULL;
  pwm.Speed = GPIO_SPEED_FREQ_LOW;
  pwm.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &pwm);
  pwm.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &pwm);

  Gimbal_MotorStop();
  /* RM0008: TIM3/TIM4 ITR1 receive TIM2 TRGO. Arm both slaves first.
   * All use the same 72 MHz clock/period; hardware start aligns carriers.
   */
  htim2.Instance->CNT = 0U;
  htim3.Instance->CNT = 0U;
  htim4.Instance->CNT = 0U;
  htim4.Instance->SMCR = TIM_TS_ITR1 | TIM_SLAVEMODE_TRIGGER;
  htim3.Instance->SMCR = TIM_TS_ITR1 | TIM_SLAVEMODE_TRIGGER;
  htim2.Instance->CR2 = (htim2.Instance->CR2 & ~TIM_CR2_MMS) | TIM_TRGO_ENABLE;
  if (__HAL_TIM_GET_AUTORELOAD(&htim3) != 3599U || htim3.Instance->PSC != 0U ||
      __HAL_TIM_GET_AUTORELOAD(&htim2) != 3599U || htim2.Instance->PSC != 0U ||
      __HAL_TIM_GET_AUTORELOAD(&htim4) != 3599U || htim4.Instance->PSC != 0U ||
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
    Gimbal_MotorStop();
    return 0;
  }
  return Gimbal_MotorHealthy();
}
int Gimbal_MotorHealthy(void)
{
  return (TIM2->CR1&TIM_CR1_CEN) && (TIM3->CR1&TIM_CR1_CEN) && (TIM4->CR1&TIM_CR1_CEN)
    && (TIM2->CCER&0x1111U)==0x1110U && (TIM3->CCER&0x1111U)==0x1111U && (TIM4->CCER&0x1111U)==0x1100U
    && TIM2->SMCR==0 && (TIM2->CR2&TIM_CR2_MMS)==TIM_TRGO_ENABLE
    && TIM3->SMCR==(TIM_TS_ITR1|TIM_SLAVEMODE_TRIGGER) && TIM4->SMCR==(TIM_TS_ITR1|TIM_SLAVEMODE_TRIGGER)
    && TIM2->PSC==0 && TIM3->PSC==0 && TIM4->PSC==0
    && TIM2->ARR==3599 && TIM3->ARR==3599 && TIM4->ARR==3599;
}
int Gimbal_MotorArm(void)
{
  uint32_t irq=__get_PRIMASK(); __disable_irq();
  int ok=Gimbal_MotorHealthy() && (GPIOC->IDR&GPIO_PIN_3) && gimbal.fault==GIMBAL_OK;
  if(ok) {
    /* Stop generated separate update events; align all carriers before arming. */
    TIM2->CR1&=~TIM_CR1_CEN; TIM3->CR1&=~TIM_CR1_CEN; TIM4->CR1&=~TIM_CR1_CEN;
    TIM2->CNT=TIM3->CNT=TIM4->CNT=0;
    TIM2->CR1|=TIM_CR1_CEN;
    armed=1;
  }
  __set_PRIMASK(irq); return ok;
}
int Gimbal_MotorApply(const float phase[3], const uint16_t power[3])
{
  uint16_t c[3][3];
  for(int i=0;i<3;i++) { if(!isfinite(phase[i])||power[i]>400) return 0; Gimbal_PWM(phase[i],power[i],c[i]); }
  uint32_t irq=__get_PRIMASK(); __disable_irq();
  /* An ISR stop cannot be undone by the interrupted main-loop write. */
  if(!armed) { __set_PRIMASK(irq); return 0; }
  /* Control arrays are roll,pitch,yaw. Actual wiring, confirmed by user:
   * MOT0=yaw (TIM3 CH2/3/4), MOT1=pitch (TIM3 CH1 + TIM2 CH4/3),
   * MOT2=roll (TIM4 CH4 + TIM2 CH2 + TIM4 CH3).
   */
  TIM3->CCR2=c[2][0]; TIM3->CCR3=c[2][1]; TIM3->CCR4=c[2][2];
  TIM3->CCR1=c[1][0]; TIM2->CCR4=c[1][1]; TIM2->CCR3=c[1][2];
  TIM4->CCR4=c[0][0]; TIM2->CCR2=c[0][1]; TIM4->CCR3=c[0][2];
  for(int i=0;i<3;i++) for(int j=0;j<3;j++) gimbal.compare[i][j]=c[i][j];
  __set_PRIMASK(irq); return 1;
}
