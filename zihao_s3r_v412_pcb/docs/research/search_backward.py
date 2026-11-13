# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import argparse,json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from env412 import ForwardEnv
from search_heading import heading_action
P=Path(__file__).resolve().parent

def trial(args):
 params,seeds=args;values=[]
 for seed in seeds:
  w=ForwardEnv(command='backward');w.reset(seed=seed)
  for _ in range(1600):
   _,_,t,tr,i=w.step(heading_action(params[:5],params[5:],w.e))
   if t or tr:break
  score=1000*i['forward_m']+800*i['late_forward_m']-1500*abs(i['cross_m'])-150*min(abs(i['yaw_rad']),3)+100*i['success']-300*i['stood']
  if i['tip_down']<.6 or i['tip_contact']<.4:score=min(score,0)-300*(max(.6-i['tip_down'],0)+max(.4-i['tip_contact'],0))
  values.append(score);w.close()
 return float(.7*np.mean(values)+.3*np.min(values))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--seed',type=int,required=True);a=p.parse_args();lo=np.array([20,-60,25,-np.pi,1.2,-.8,-.25,-.8,-.25]);hi=np.array([60,15,80,np.pi,3.2,.8,.25,.8,.25]);rng=np.random.default_rng(a.seed)
 mean=np.array([50,-15,40,2.2,1.5,0,0,0,0.]);std=(hi-lo)/4;best=None;history=[];name=f'backward_heading_{a.seed}'
 with ProcessPoolExecutor(max_workers=4) as pool:
  for gen in range(12):
   pop=np.clip(rng.normal(mean,std,(40,9)),lo,hi);pop[0]=np.r_[best['params'],best['gains']] if best else mean
   scores=list(pool.map(trial,[(v,[92000000,92000001,92000002]) for v in pop]));order=np.argsort(scores)[::-1];elite=pop[order[:8]];mean=.2*mean+.8*elite.mean(0);std=np.maximum(.2*std+.8*elite.std(0),(hi-lo)*.05)
   if best is None or scores[order[0]]>best['score']:best=dict(params=pop[order[0]][:5].tolist(),gains=pop[order[0]][5:].tolist(),score=scores[order[0]])
   history.append(dict(generation=gen,best=best));(P/(name+'_progress.json')).write_text(json.dumps(history,indent=2));print(name,gen,best,flush=True)
 (P/(name+'.json')).write_text(json.dumps(dict(author='同济子豪兄 2026-10-26',kind='heading',seed=a.seed,best=best,history=history),indent=2))
