# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import argparse,json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from env412 import ForwardEnv
P=Path(__file__).resolve().parent

def decode(kind,params,t,target):
 if kind=='sin':
  amp,offset,upper,phase,period=params;theta=2*np.pi*t/period;desired=np.deg2rad([amp*np.sin(theta),offset+upper*np.sin(theta+phase)])
 else:
  knots=np.array(params[:8]).reshape(4,2);knots[:,0]-=knots[:,0].mean();knots[:,0]*=min(1,60/max(np.max(abs(knots[:,0])),1e-8));period=params[-1];phase=(t/period%1)*4;i=int(phase);alpha=phase-i;desired=np.deg2rad(knots[i]*(1-alpha)+knots[(i+1)%4]*alpha)
 desired=np.clip(desired,[-np.pi/3,-2*np.pi/3],[np.pi/3,2*np.pi/3])
 return np.clip(desired-target,[-.06,-.08],[.06,.08])/.1

def rollout(kind,params,seed):
 w=ForwardEnv();o,_=w.reset(seed=seed);e=w.e
 for _ in range(1600):
  _,_,t,tr,i=w.step(decode(kind,params,e.elapsed,e.target))
  if t or tr:break
 # Include startup displacement; reward later progress to reject one-off lurches.
 score=1000*i['forward_m']+1500*i['late_forward_m']-600*abs(i['cross_m'])-40*min(abs(i['yaw_rad']),3)+8*i['tip_down']+5*i['tip_contact']-300*i['stood']+60*i['success']
 if i['forward_m']>.02:score+=min(i['lower_span_deg'],100)*.05+min(i['upper_span_deg'],180)*.025
 if not i['valid']:score-=1000
 w.close();return float(score),dict(seed=seed,**i)
def candidate(args):
 kind,params,seeds,*flags=args;rows=[rollout(kind,params,s) for s in seeds];r=[]
 for score,info in rows:
  if flags and flags[0] and (info['tip_down']<.6 or info['tip_contact']<.4):score=min(score,0)-300*(max(.6-info['tip_down'],0)+max(.4-info['tip_contact'],0))
  r.append(score)
 return float(.7*np.mean(r)+.3*np.min(r))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--kind',required=True);p.add_argument('--seed',type=int,required=True);p.add_argument('--grip',action='store_true');a=p.parse_args();kind=a.kind
 lo=np.array([20,-70,30,-np.pi,1.2]) if kind=='sin' else np.array([-60,-110]*4+[1.4]);hi=np.array([60,30,100,np.pi,4.]) if kind=='sin' else np.array([60,100]*4+[4.])
 rng=np.random.default_rng(a.seed);mean=(lo+hi)/2;std=(hi-lo)/3;best=None;history=[];seeds=[82000000,82000001];name=f'forward_{kind}_'+('grip_' if a.grip else '')+str(a.seed)
 if a.grip and (P/'forward_sin_2301.json').exists():mean=np.array(json.loads((P/'forward_sin_2301.json').read_text())['best']['params']);mean[1]=-30
 with ProcessPoolExecutor(max_workers=4) as pool:
  for gen in range(10):
   pop=np.clip(rng.normal(mean,std,(32,len(lo))),lo,hi)
   if best:pop[0]=best['params']
   scores=list(pool.map(candidate,[(kind,p,seeds,a.grip) for p in pop]));order=np.argsort(scores)[::-1];elite=pop[order[:6]];mean=.2*mean+.8*elite.mean(0);std=np.maximum(.2*std+.8*elite.std(0),(hi-lo)*.05)
   if best is None or scores[order[0]]>best['score']:best=dict(params=pop[order[0]].tolist(),score=float(scores[order[0]]))
   history.append(dict(generation=gen,best=best,mean_score=float(np.mean(scores))));(P/(name+'_progress.json')).write_text(json.dumps(history,indent=2));print(name,gen,best,flush=True)
 (P/(name+'.json')).write_text(json.dumps(dict(author='同济子豪兄 2026-10-26',kind=kind,seed=a.seed,best=best,history=history,training_seeds=seeds),indent=2))
