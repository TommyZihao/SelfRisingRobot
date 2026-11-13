# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import sys
from collections import deque
import numpy as np,gymnasium as gym
P=Path(__file__).resolve().parent
sys.path.insert(0,str(P.parent/'walk_continuous_roll'))
from continuous_env import ContinuousRoll
from ground_env import GroundRobot,contact_state,tip_orientation
class RightEnv(ContinuousRoll):
 def __init__(self,horizon=800):super().__init__(horizon=horizon)
 def reset(self,seed=None,options=None):
  o,i=super().reset(seed=seed,options=dict(options or {},command='roll_right'));self.first_hold=0;self.first_pass=False;self.first_time=None;return o,self.info()
 def step(self,a):
  before=self.net;q=self.quarters;goal=self.e.roll_goal.copy();old=float(self.e.R()[2]@goal)
  o,_,t,tr,i=super().step(a);e=self.e;inc=self.net-before
  r=30*np.clip(inc,-.15,.15)+8*(float(e.R()[2]@goal)-old)+20*(self.quarters-q)
  r-=40*max(e.top_height-.12,0)+30*max(e.com_height-.045,0)+4*max(abs(e.R()[2,2])-.5,0)
  r-=.001*np.sum(e.prev_action**2)+.02*float(self.count-self.last_quarter_step>150)
  self.first_hold=self.first_hold+1 if self.net>=.24*2*np.pi else 0
  if self.first_hold>=10 and not self.first_pass:self.first_pass=True;self.first_time=self.count*.02;r+=15
  if t:r-=30
  return o,float(r),t,tr,self.info()
 def info(self):
  i=super().info();i.update(first_roll=bool(getattr(self,'first_pass',False) and not self.stood and not self.invalid),first_time=getattr(self,'first_time',None));return i
class ForwardEnv(gym.Env):
 def __init__(self,horizon=1600,constrain=True,command="forward"):
  self.command=command
  self.e=GroundRobot('carpet_proxy',commands=[command],poses=['pitch_neg'],strict=False,horizon=horizon);self.horizon=horizon;self.constrain=constrain;self.action_space=self.e.action_space;self.observation_space=self.e.observation_space
 def reset(self,seed=None,options=None):
  for attempt in range(100):
   o,i=self.e.reset(seed=None if seed is None else seed+1000003*attempt,options={'pose':'pitch_neg','command':self.command})
   if self.e.initial_valid and self.e.actual_pose=='pitch_neg':break
  self.count=0;self.last=self.e.data.subtree_com[self.e.body].copy();self.last_window=0.;self.windows=[];self.targets=deque(maxlen=100);self.contacts=0;self.down=0;self.stood=False;self.valid=True;self.max_penetration=0.;self.lower=[];self.upper=[]
  return o,self.info()
 def step(self,a):
  e=self.e;a=np.clip(a,-1,1)
  if self.constrain:
   desired=np.clip(e.target+.1*a,[-np.pi/3,-e.limit[1]],[np.pi/3,e.limit[1]]);a=np.clip(desired-e.target,[-.06,-.08],[.06,.08])/.1
  o,_,_,_,base=e.step(a);self.count+=1
  self.valid &=base['valid'];self.stood|=bool(e.top_height>.145 and e.com_height>.037)
  pos=e.data.subtree_com[e.body].copy();v=(pos-self.last)/.02;self.last=pos;c=contact_state(e);up=tip_orientation(e)
  self.max_penetration=max(self.max_penetration,c['penetration_m']);self.contacts+=int(c['contact']);self.down+=int(up<=-.1)
  self.targets.append(e.target.copy());self.lower.append(float(e.target[0]));self.upper.append(float(e.target[1]))
  speed=float(v@e.goal_world);progress=float((pos-e.origin)@e.goal_world)
  span=np.ptp(np.array(self.targets),axis=0) if len(self.targets)>1 else np.zeros(2)
  # Bigger strokes only earn a bonus while producing forward translation.
  amplitude=min(float(np.sum(span)),3.)*max(min(speed,.03),0)*8
  r=250*np.clip(speed,-.06,.06)+amplitude+.2*float(c['contact'])-.8*max(up,0)
  r-=15*min(abs(float(np.cross(v,e.goal_world)[2])),.1)+.2*min(abs(e.yaw_progress),3)
  r-=.2*np.sum(np.maximum(abs(e.target/e.limit)-.9,0))+.002*np.sum(a*a)
  r-=30*max(e.top_height-.12,0)+20*max(e.com_height-.045,0)
  if self.count%100==0:self.windows.append(progress-self.last_window);self.last_window=progress
  t=not self.valid or self.stood
  if t:r-=30
  return o,float(r),bool(t),self.count>=self.horizon,self.info()
 def info(self):
  e=self.e;delta=e.data.subtree_com[e.body]-e.origin;forward=float(delta@e.goal_world);cross=float(np.cross(delta,e.goal_world)[2]);late=sum(self.windows[8:]);sustained=len(self.windows)==16 and sum(x>=.002 for x in self.windows[8:])>=6
  return dict(forward_m=forward,cross_m=cross,yaw_rad=float(e.yaw_progress),late_forward_m=late,windows=self.windows.copy(),time_s=self.count*.02,stood=self.stood,valid=self.valid,tip_contact=self.contacts/max(self.count,1),tip_down=self.down/max(self.count,1),max_penetration_m=self.max_penetration,lower_span_deg=float(np.rad2deg(np.ptp(self.lower[100:]))) if len(self.lower)>100 else 0.,upper_span_deg=float(np.rad2deg(np.ptp(self.upper[100:]))) if len(self.upper)>100 else 0.,success=bool(forward>=.04 and sustained and abs(cross)<max(.02,forward) and abs(e.yaw_progress)<np.pi/4 and self.down/max(self.count,1)>.6 and self.contacts/max(self.count,1)>.4 and not self.stood and self.valid),actual_start=e.actual_pose)
 def close(self):self.e.close()
