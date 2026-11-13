// 作者：同济子豪兄 2026-10-26 | RoBoBo 4.1.2-PCB
#pragma once
#include <math.h>
#include "walk_config.h"
#include "walk41_legacy.h"
#include "walk41_roll.h"
#include "walk412_right.h"
#include "walk412_forward_config.h"
#if FORWARD412_KIND == 2
#include "walk412_forward.h"
#endif
#include "walk412_backward_config.h"
#if BACKWARD412_KIND == 2
#include "walk412_backward.h"
#endif
namespace walk41 {
constexpr float pi=3.14159265358979323846f;
constexpr float limit[2]={pi/2,2*pi/3};
inline float clip(float x,float a,float b){return fmaxf(a,fminf(b,x));}
enum Command {Idle,Forward,Backward,RollLeft,RollRight,YawLeft,YawRight};
inline void basis(const float* u,float* f){
 for(int i=0;i<3;i++)f[i]=(i==2?1.f:0.f)-u[i]*u[2];
 float n=sqrtf(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);
 if(n<.15f){for(int i=0;i<3;i++)f[i]=(i==0?1.f:0.f)-u[i]*u[0];n=sqrtf(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);}
 for(int i=0;i<3;i++)f[i]/=fmaxf(n,1.e-8f);
}
struct Controller {
 Command command=Idle;double elapsed=0,net=0;int quarters=0;
 float previous[2]={0,0},goalWorld[3]={0,0,0},initialUp[3]={0,0,1};
 float initialAngle=0,lastAngle=0;bool first=true;
 void cancel(){command=Idle;previous[0]=previous[1]=0;elapsed=0;net=0;quarters=0;first=true;}
 void setCommand(Command next,const float* R){
  if(next==command)return; // Heartbeats preserve phase, progress, and previous action.
  command=next;elapsed=0;net=0;quarters=0;first=true;
  for(int i=0;i<3;i++){initialUp[i]=R[6+i];goalWorld[i]=0;}
  initialAngle=lastAngle=atan2f(R[7],R[6]);
  if(next==Idle){previous[0]=previous[1]=0;return;}
  if(next==Backward||next==Forward){float f[3];basis(R+6,f);for(int i=0;i<3;i++)goalWorld[i]=(next==Backward?-1.f:1.f)*(R[3*i]*f[0]+R[3*i+1]*f[1]+R[3*i+2]*f[2]);goalWorld[2]=0;float n=hypotf(goalWorld[0],goalWorld[1]);for(int i=0;i<3;i++)goalWorld[i]/=fmaxf(n,1.e-8f);}
 }
 bool step(const float* R,const float* gyro,float* targets){
  for(int i=0;i<9;i++)if(!isfinite(R[i]))return false;
  for(int i=0;i<3;i++)if(!isfinite(gyro[i]))return false;
  for(int i=0;i<2;i++)if(!isfinite(targets[i]))return false;
  if(command==Idle)return true;
  float action[2]={0,0},desired[2]={0,0};const float* u=R+6;
  if(command==Forward){
#if FORWARD412_KIND == 2
   float obs[22]={0};
   for(int i=0;i<3;i++){obs[i]=u[i];obs[3+i]=gyro[i]/10;obs[6+i]=R[i]*goalWorld[0]+R[3+i]*goalWorld[1]+R[6+i]*goalWorld[2];}
   for(int i=0;i<2;i++){obs[14+i]=targets[i]/limit[i];obs[16+i]=previous[i];}
   obs[18]=sin(2*3.14159265358979323846*elapsed/1.6);obs[19]=cos(2*3.14159265358979323846*elapsed/1.6);obs[21]=fmin(elapsed/10,1.);
   for(int i=0;i<22;i++)obs[i]=clip(obs[i],-5,5);
   walk412_forward::forward(obs,action);
   for(int i=0;i<2;i++)desired[i]=targets[i]+.1f*action[i];
#elif FORWARD412_KIND == 1
   double knots[4][2],mean=0,maximum=0;
   for(int i=0;i<4;i++){knots[i][0]=FORWARD412_PARAMS[2*i];knots[i][1]=FORWARD412_PARAMS[2*i+1];mean+=knots[i][0]/4;}
   for(int i=0;i<4;i++){knots[i][0]-=mean;maximum=fmax(maximum,fabs(knots[i][0]));}
   double scale=fmin(1.,60/fmax(maximum,1.e-8));for(int i=0;i<4;i++)knots[i][0]*=scale;
   double phase=fmod(elapsed/FORWARD412_PARAMS[8],1.)*4;int k=(int)phase;double a=phase-k;
   for(int i=0;i<2;i++)desired[i]=(knots[k][i]*(1-a)+knots[(k+1)%4][i]*a)*pi/180;
#else
   double phase=2*3.14159265358979323846*elapsed/FORWARD412_PARAMS[4];
   desired[0]=FORWARD412_PARAMS[0]*sin(phase)*pi/180;
   desired[1]=(FORWARD412_PARAMS[1]+FORWARD412_PARAMS[2]*sin(phase+FORWARD412_PARAMS[3]))*pi/180;
#if FORWARD412_KIND == 3
   float f[3],front[3]={0,0,0};basis(u,f);
   for(int i=0;i<3;i++)for(int j=0;j<3;j++)front[i]+=R[i*3+j]*f[j];
   float error=atan2f(front[0]*goalWorld[1]-front[1]*goalWorld[0],front[0]*goalWorld[0]+front[1]*goalWorld[1]+front[2]*goalWorld[2]);
   float rate=gyro[0]*u[0]+gyro[1]*u[1]+gyro[2]*u[2];
   for(int i=0;i<2;i++)desired[i]+=clip(FORWARD412_GAINS[2*i]*error+FORWARD412_GAINS[2*i+1]*rate,-.35f,.35f);
#endif
#endif
   desired[0]=clip(desired[0],-pi/3,pi/3);desired[1]=clip(desired[1],-limit[1],limit[1]);
   action[0]=clip(desired[0]-targets[0],-.06f,.06f)/.1f;
   action[1]=clip(desired[1]-targets[1],-.08f,.08f)/.1f;
  }else if(command==Backward){
#if BACKWARD412_KIND == 2
   float obs[22]={0};
   for(int i=0;i<3;i++){obs[i]=u[i];obs[3+i]=gyro[i]/10;obs[6+i]=R[i]*goalWorld[0]+R[3+i]*goalWorld[1]+R[6+i]*goalWorld[2];}
   for(int i=0;i<2;i++){obs[14+i]=targets[i]/limit[i];obs[16+i]=previous[i];}
   obs[18]=sin(2*3.14159265358979323846*elapsed/1.6);obs[19]=cos(2*3.14159265358979323846*elapsed/1.6);obs[21]=fmin(elapsed/10,1.);
   for(int i=0;i<22;i++)obs[i]=clip(obs[i],-5,5);
   walk412_backward::forward(obs,action);
   for(int i=0;i<2;i++)desired[i]=targets[i]+.1f*action[i];
#elif BACKWARD412_KIND == 1
   double knots[4][2],mean=0,maximum=0;
   for(int i=0;i<4;i++){knots[i][0]=BACKWARD412_PARAMS[2*i];knots[i][1]=BACKWARD412_PARAMS[2*i+1];mean+=knots[i][0]/4;}
   for(int i=0;i<4;i++){knots[i][0]-=mean;maximum=fmax(maximum,fabs(knots[i][0]));}
   double scale=fmin(1.,60/fmax(maximum,1.e-8));for(int i=0;i<4;i++)knots[i][0]*=scale;
   double phase=fmod(elapsed/BACKWARD412_PARAMS[8],1.)*4;int k=(int)phase;double a=phase-k;
   for(int i=0;i<2;i++)desired[i]=(knots[k][i]*(1-a)+knots[(k+1)%4][i]*a)*pi/180;
#else
   double phase=2*3.14159265358979323846*elapsed/BACKWARD412_PARAMS[4];
   desired[0]=BACKWARD412_PARAMS[0]*sin(phase)*pi/180;
   desired[1]=(BACKWARD412_PARAMS[1]+BACKWARD412_PARAMS[2]*sin(phase+BACKWARD412_PARAMS[3]))*pi/180;
#if BACKWARD412_KIND == 3
   float f[3],front[3]={0,0,0};basis(u,f);
   for(int i=0;i<3;i++)for(int j=0;j<3;j++)front[i]+=R[i*3+j]*f[j];
   float error=atan2f(-front[0]*goalWorld[1]+front[1]*goalWorld[0],-front[0]*goalWorld[0]-front[1]*goalWorld[1]-front[2]*goalWorld[2]);
   float rate=gyro[0]*u[0]+gyro[1]*u[1]+gyro[2]*u[2];
   for(int i=0;i<2;i++)desired[i]+=clip(BACKWARD412_GAINS[2*i]*error+BACKWARD412_GAINS[2*i+1]*rate,-.35f,.35f);
#endif
#endif
   desired[0]=clip(desired[0],-pi/3,pi/3);desired[1]=clip(desired[1],-limit[1],limit[1]);
   action[0]=clip(desired[0]-targets[0],-.06f,.06f)/.1f;
   action[1]=clip(desired[1]-targets[1],-.08f,.08f)/.1f;
  }else if(command==RollLeft||command==RollRight){
   int direction=command==RollLeft?-1:1;float angle=atan2f(u[1],u[0]);
   if(!first&&hypotf(u[0],u[1])>.6f)net-=direction*atan2f(sinf(angle-lastAngle),cosf(angle-lastAngle));
   lastAngle=angle;int count=(int)((net+1.e-8)/(pi/2));if(count>quarters)quarters=count;
   float goal[3],obs[22]={0};
   if(first){goal[0]=direction*initialUp[1];goal[1]=-direction*initialUp[0];goal[2]=initialUp[2];}
   else{double a=initialAngle-direction*(quarters+1)*3.14159265358979323846/2;goal[0]=cos(a);goal[1]=sin(a);goal[2]=0;}
   for(int i=0;i<3;i++){obs[i]=u[i];obs[3+i]=gyro[i]/10;obs[11+i]=goal[i];}
   obs[10]=direction;
   for(int i=0;i<2;i++){obs[14+i]=targets[i]/limit[i];obs[16+i]=previous[i];}
   double phase=2*3.14159265358979323846*elapsed/1.6;
   obs[18]=sin(phase);obs[19]=cos(phase);obs[20]=(net-quarters*3.14159265358979323846/2)/pi;obs[21]=fmin(elapsed/10,1.);
   for(int i=0;i<22;i++)obs[i]=clip(obs[i],-5,5);
   if(command==RollRight)walk412_right::forward(obs,action);else walk41_roll::forward(obs,action);
  }else{
   int pose=fabsf(u[0])>=fabsf(u[1])?(u[0]>=0?0:1):(u[1]>=0?2:3);
   int gait=pose*6+(command==Backward?1:command==YawLeft?4:5);
   const double* g=WALK_GAITS[gait];double phase=2*3.14159265358979323846*elapsed/g[5];
   float obs[16]={0},residual[2];
   for(int i=0;i<3;i++){obs[i]=u[i];obs[3+i]=R[i]*goalWorld[0]+R[3+i]*goalWorld[1]+R[6+i]*goalWorld[2];obs[6+i]=clip(gyro[i]/10,-3,3);}
   for(int i=0;i<2;i++){obs[9+i]=targets[i]/limit[i];obs[11+i]=previous[i];}
   obs[13]=sin(phase);obs[14]=cos(phase);obs[15]=command==YawLeft?1:command==YawRight?-1:0;
   walk41_legacy::forward(obs,residual);
   for(int i=0;i<2;i++){desired[i]=limit[i]*clip(g[i]+g[2+i]*sin(phase+(i?g[4]:0))+(pose==0?.25f:0.f)*residual[i],-1,1);action[i]=clip((desired[i]-targets[i])/.1f,-1,1);}
  }
  for(int i=0;i<2;i++){
   if(!isfinite(action[i]))return false;
   previous[i]=clip(action[i],-1,1);targets[i]=clip(targets[i]+.1f*previous[i],-limit[i],limit[i]);
  }
  first=false;elapsed+=.02;return true;
 }
};
}
