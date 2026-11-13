# 作者：同济子豪兄 2026-10-26
import argparse,ctypes,subprocess,tempfile,json,sys
from pathlib import Path
import numpy as np,torch
from stable_baselines3 import PPO
p=argparse.ArgumentParser();p.add_argument('--models',type=Path,required=True);p.add_argument('--rl',type=Path);a=p.parse_args();root=Path(__file__).resolve().parents[1];torch.set_num_threads(1)
code='''#include "walk41_controller.h"
#include "recovery_network.h"
static walk41::Controller c;
extern "C" {
void reset(){c=walk41::Controller();}
void command(int n,const float* r){c.setCommand((walk41::Command)n,r);}
int tick(const float* r,const float* g,float* t){return c.step(r,g,t);}
void recovery(const float* x,float* y){recovery_policy::forward_policy(x,y);for(int i=0;i<2;i++)y[i]=walk41::clip(y[i],-1,1);}
void legacy(const float* x,float* y){walk41_legacy::forward(x,y);}
void roll(const float* x,float* y){walk41_roll::forward(x,y);}
void right(const float* x,float* y){walk412_right::forward(x,y);}
double elapsed(){return c.elapsed;}
}
'''
fptr=ctypes.POINTER(ctypes.c_float)
def ptr(x):return x.ctypes.data_as(fptr)
with tempfile.TemporaryDirectory() as temp:
 src=Path(temp)/'test.cpp';src.write_text(code);lib=Path(temp)/'test.dylib'
 subprocess.run(['c++','-std=c++17','-O2','-shared','-fPIC','-I',str(root),str(src),'-o',str(lib)],check=True)
 c=ctypes.CDLL(str(lib));c.command.argtypes=[ctypes.c_int,fptr];c.tick.argtypes=[fptr,fptr,fptr];c.elapsed.restype=ctypes.c_double
 errors={}
 for file,fn,dim in [('recovery_ppo.zip','recovery',4),('legacy_walk.zip','legacy',16),('continuous_roll.zip','roll',22),('right_roll.zip','right',22)]:
  m=PPO.load(a.models/file,device='cpu');x=np.random.default_rng(42).uniform(-3,3,(1000,dim)).astype('float32');expected=m.predict(x,deterministic=True)[0];out=[]
  for row in x:
   y=np.zeros(2,dtype='float32');getattr(c,fn)(ptr(row),ptr(y));out.append(y)
  err=float(np.max(abs(expected-np.array(out))));assert err<2e-5,(fn,err);errors[fn+'_network']=err
 if a.rl is None:
  print(json.dumps(errors,indent=2));raise SystemExit(0)
 sys.path.insert(0,str(a.rl/'walk_continuous_roll'));from continuous_env import ContinuousRoll
 sys.path.insert(0,str(a.rl/'walk_v412'));from env412 import ForwardEnv
 from eval412 import Actor412
 selected=json.loads((a.rl/'walk_v412/selection.json').read_text())
 from ground_env import GroundRobot,Actor
 def inputs(e):return e.R().astype('float32').ravel(),e.data.qvel[3:6].astype('float32'),e.target.astype('float32').copy()
 right_model=PPO.load(a.models/'right_roll.zip',device='cpu')
 for command,number in [('roll_left',3),('roll_right',4)]:
  w=ContinuousRoll();obs,_=w.reset(seed=74000000,options={'command':command});e=w.e;e.randomize=False;c.reset();R,g,t=inputs(e);c.command(number,ptr(R));model=PPO.load(a.models/'continuous_roll.zip',device='cpu');maximum=0.
  for k in range(1200):
   if k in [400,450,850,900]:
    name='idle' if k in [400,850] else ('roll_right' if command=='roll_left' else 'roll_left') if k==450 else command
    obs=w.set_command(name);R,g,t=inputs(e);c.command({'idle':0,'roll_left':3,'roll_right':4}[name],ptr(R))
   R,g,t=inputs(e)
   if k%4==0:c.command({'idle':0,'roll_left':3,'roll_right':4}[e.command],ptr(R))
   assert c.tick(ptr(R),ptr(g),ptr(t))
   active_model=right_model if e.command=='roll_right' else model
   action=active_model.predict(e.observe(),deterministic=True)[0];w.step(action)
   err=float(np.max(abs(t-e.target)));maximum=max(maximum,err)
   assert err<2e-4,(command,k,err,t,e.target)
  errors[command+'_switching']=maximum;w.close()
 for command,number in [('forward',1),('backward',2),('yaw_left',5),('yaw_right',6)]:
  w=ForwardEnv(command=command) if command in ['forward','backward'] else None
  if w:o,_=w.reset(seed=66000024);e=w.e;actor412=Actor412(selected[command])
  else:e=GroundRobot('carpet_proxy',strict=False);e.reset(seed=38000100,options={'pose':'pitch_neg','command':command});actor=Actor('old_firmware');actor.reset(e)
  e.randomize=False;c.reset();R,g,t=inputs(e);c.command(number,ptr(R));maximum=0.
  for k in range(1600 if w else 600):
   R,g,t=inputs(e)
   if k%4==0:c.command(number,ptr(R))
   assert c.tick(ptr(R),ptr(g),ptr(t))
   if w:o,_,_,_,_=w.step(actor412.predict(e,o))
   else:e.step(actor.predict(e,e.observe()))
   err=float(np.max(abs(t-e.target)));maximum=max(maximum,err);assert err<2e-4,(command,k,err,t,e.target)
  # Release and nonfinite input.
  R,g,t=inputs(e);c.command(0,ptr(R));before=t.copy();assert c.tick(ptr(R),ptr(g),ptr(t));assert np.array_equal(before,t)
  R[0]=np.nan;assert not c.tick(ptr(R),ptr(g),ptr(t))
  errors[command]=maximum;e.close()
 (root/'tests/parity_results.json').write_text(json.dumps(errors,indent=2));print(errors)
