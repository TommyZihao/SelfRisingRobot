# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import argparse,json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from env412 import ForwardEnv
from ground_env import GroundRobot
from base_env import basis
P=Path(__file__).resolve().parent

def heading_action(params,gains,e):
 amp,offset,upper,phase,period=params;theta=2*np.pi*e.elapsed/period
 front=e.R()@basis(e.R()[2]);heading=e.goal_world*(-1 if e.command=='backward' else 1);error=np.arctan2(np.cross(front,heading)[2],front@heading);rate=float(e.data.qvel[3:6]@e.R()[2])
 desired=np.deg2rad([amp*np.sin(theta),offset+upper*np.sin(theta+phase)])
 desired+=np.clip(np.array(gains).reshape(2,2)@np.array([error,rate]),-.35,.35)
 desired=np.clip(desired,[-np.pi/3,-e.limit[1]],[np.pi/3,e.limit[1]])
 return np.clip(desired-e.target,[-.06,-.08],[.06,.08])/.1

def trial(args):
 params,gains,seeds=args;scores=[]
 for seed in seeds:
  w=ForwardEnv();o,_=w.reset(seed=seed)
  for _ in range(1600):
   _,_,t,tr,i=w.step(heading_action(params,gains,w.e))
   if t or tr:break
  score=1000*i['forward_m']+800*i['late_forward_m']-2000*abs(i['cross_m'])-200*min(abs(i['yaw_rad']),3)+60*i['success']-300*i['stood']
  if i['tip_down']<.6 or i['tip_contact']<.4:score=min(score,0)-300*(max(.6-i['tip_down'],0)+max(.4-i['tip_contact'],0))
  scores.append(score);w.close()
 return float(.7*np.mean(scores)+.3*np.min(scores))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--seed',type=int,required=True);a=p.parse_args();params=json.loads((P/'forward_sin_grip_2402.json').read_text())['best']['params'];rng=np.random.default_rng(a.seed);lo=np.array([-.8,-.25,-.8,-.25]);hi=-lo;mean=np.zeros(4);std=(hi-lo)/3;best=None;history=[];name=f'forward_heading_{a.seed}'
 with ProcessPoolExecutor(max_workers=4) as pool:
  for gen in range(10):
   pop=np.clip(rng.normal(mean,std,(32,4)),lo,hi);pop[0]=best['gains'] if best else np.zeros(4)
   scores=list(pool.map(trial,[(params,g,[88000000,88000001,88000002]) for g in pop]));order=np.argsort(scores)[::-1];elite=pop[order[:6]];mean=.2*mean+.8*elite.mean(0);std=np.maximum(.2*std+.8*elite.std(0),(hi-lo)*.05)
   if best is None or scores[order[0]]>best['score']:best=dict(params=params,gains=pop[order[0]].tolist(),score=scores[order[0]])
   history.append(dict(generation=gen,best=best));(P/(name+'_progress.json')).write_text(json.dumps(history,indent=2));print(name,gen,best,flush=True)
 (P/(name+'.json')).write_text(json.dumps(dict(author='同济子豪兄 2026-10-26',kind='heading',seed=a.seed,best=best,history=history),indent=2))
