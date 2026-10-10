#include "gimbal_control.h"
#include <math.h>
#define RAD 0.017453292519943295f
#define DEG 57.29577951308232f
float Gimbal_Clamp(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
float Gimbal_Wrap(float x) { return x - 360.0f * floorf((x + 180.0f) / 360.0f); }
int Gimbal_ValidAxes(const int a[3])
{
  int m[3][3] = {{0}};
  for (int i=0;i<3;i++) { int j=a[i]<0?-a[i]:a[i]; if(j<1||j>3) return 0; m[i][j-1]=a[i]>0?1:-1; }
  int det=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
         -m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
         +m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
  return det==1;
}
void Gimbal_Map(const float raw[3], const int axes[3], float body[3])
{
  for(int i=0;i<3;i++) { int a=axes[i]; body[i]=(a>0?1.0f:-1.0f)*raw[(a>0?a:-a)-1]; }
}
static void angles(Gimbal_Attitude *s)
{
  const float *q=s->q;
  s->angle[0]=atan2f(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]))*DEG;
  s->angle[1]=asinf(Gimbal_Clamp(2*(q[0]*q[2]-q[3]*q[1]),-1,1))*DEG;
  s->angle[2]=atan2f(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))*DEG;
}
void Gimbal_AttitudeInit(Gimbal_Attitude *s, const float a[3])
{
  /* Accelerometer measures specific force: negate it to obtain gravity down. */
  float r=atan2f(-a[1],-a[2]), p=atan2f(a[0],sqrtf(a[1]*a[1]+a[2]*a[2]));
  float cr=cosf(r/2),sr=sinf(r/2),cp=cosf(p/2),sp=sinf(p/2);
  s->q[0]=cr*cp;s->q[1]=sr*cp;s->q[2]=cr*sp;s->q[3]=-sr*sp;
  for(int i=0;i<3;i++) s->rate[i]=0;
  angles(s);
}
int Gimbal_AttitudeUpdate(Gimbal_Attitude *s, const float a[3], const float gyro[3], float dt)
{
  if(!isfinite(dt)||dt<=0||dt>0.025f) return 0;
  float *q=s->q, w[3], norm=0;
  for(int i=0;i<3;i++) { if(!isfinite(a[i])||!isfinite(gyro[i])) return 0; w[i]=gyro[i]*RAD; norm+=a[i]*a[i]; }
  /* Proportional gravity correction; no magnetometer means yaw is relative. */
  if(norm>0.64f && norm<1.44f) {
    norm=sqrtf(norm);
    float g[3]={-a[0]/norm,-a[1]/norm,-a[2]/norm};
    float v[3]={2*(q[1]*q[3]-q[0]*q[2]),2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2])};
    w[0]+=2*(g[1]*v[2]-g[2]*v[1]); w[1]+=2*(g[2]*v[0]-g[0]*v[2]); w[2]+=2*(g[0]*v[1]-g[1]*v[0]);
  }
  float h=dt/2,old[4]={q[0],q[1],q[2],q[3]};
  q[0]+=h*(-old[1]*w[0]-old[2]*w[1]-old[3]*w[2]);
  q[1]+=h*( old[0]*w[0]+old[2]*w[2]-old[3]*w[1]);
  q[2]+=h*( old[0]*w[1]-old[1]*w[2]+old[3]*w[0]);
  q[3]+=h*( old[0]*w[2]+old[1]*w[1]-old[2]*w[0]);
  norm=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  if(!isfinite(norm)||norm<0.1f) return 0;
  for(int i=0;i<4;i++) q[i]/=norm;
  angles(s);
  /* Convert body gyro to Euler angle rates, for derivative damping. */
  float r=s->angle[0]*RAD,p=s->angle[1]*RAD,cr=cosf(r),sr=sinf(r),cp=cosf(p);
  if(fabsf(cp)<0.1f) return 0;
  float t=sr*gyro[1]+cr*gyro[2];
  s->rate[0]=gyro[0]+t*tanf(p); s->rate[1]=cr*gyro[1]-sr*gyro[2]; s->rate[2]=t/cp;
  return 1;
}
float Gimbal_PIDStep(Gimbal_PID *p, float error, float rate, float dt, int direction)
{
  /* PID output is electrical phase speed (deg/s), not mechanical position.
   * Empirical gains absorb pole count; phase integration moves the magnetic field.
   * Conditional integration prevents windup when the phase-speed limit saturates.
   */
  float next=Gimbal_Clamp(p->integral+error*dt,-30,30);
  float out=p->kp*error+p->ki*next-p->kd*rate;
  if(fabsf(out)<600 || out*error<0) p->integral=next;
  out=Gimbal_Clamp(p->kp*error+p->ki*p->integral-p->kd*rate,-600,600);
  p->phase=Gimbal_Wrap(p->phase+direction*out*dt);
  return p->phase;
}
void Gimbal_PWM(float phase, unsigned strength, uint16_t c[3])
{
  if(strength==0) { c[0]=c[1]=c[2]=0; return; }
  if(strength>400) strength=400;
  float s=sinf(phase*RAD),co=cosf(phase*RAD),amp=1.8f*strength;
  c[0]=(uint16_t)(1800+amp*s);
  c[1]=(uint16_t)(1800+amp*(-0.5f*s+0.8660254038f*co));
  c[2]=(uint16_t)(1800+amp*(-0.5f*s-0.8660254038f*co));
}
