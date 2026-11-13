# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import argparse,json
import numpy as np,torch
from stable_baselines3 import PPO
from env412 import RightEnv,ForwardEnv
from search_forward import decode
P=Path(__file__).resolve().parent;torch.set_num_threads(1)
class Actor412:
 def __init__(self,name):
  self.name=name;self.model=None;self.gait=None
  if name=='backward_41':
   from ground_env import Actor
   self.legacy=Actor('old_firmware')
  elif name=='right_41':self.model=PPO.load(P.parent/'walk_continuous_roll/runs/warm_1101/final.zip',device='cpu')
  elif name=='forward_41':self.gait=json.loads((P.parent/'walk_forward_center/gait_search32.json').read_text())['best']['params']
  elif (P/(name+'.json')).exists():
   d=json.loads((P/(name+'.json')).read_text());self.gait=d['best']['params'];self.kind=d['kind'];self.gains=d['best'].get('gains')
  else:self.model=PPO.load(P/'runs'/name/'final.zip',device='cpu')
 def predict(self,e,o):
  if self.name=='backward_41':return self.legacy.predict(e,o)
  if self.model is not None:return self.model.predict(o,deterministic=True)[0]
  if self.name=='forward_41':
   amp,offset,upper,phase,period=self.gait;theta=2*np.pi*e.elapsed/period;desired=np.deg2rad([amp*np.sin(theta),offset+upper*np.sin(theta+phase)]);desired=np.clip(desired,[-np.pi/6,-e.limit[1]],[np.pi/6,e.limit[1]]);return np.clip(desired-e.target,[-.03,-.04],[.03,.04])/.1
  if self.kind=='heading':
   from search_heading import heading_action
   return heading_action(self.gait,self.gains,e)
  return decode(self.kind,self.gait,e.elapsed,e.target)
def episode(name,seed,frame=None):
 actor=Actor412(name) if isinstance(name,str) else name;name=actor.name;task='right' if name.startswith('right') else 'backward' if name.startswith('backward') else 'forward';w=RightEnv(horizon=1200) if task=='right' else ForwardEnv(constrain=name not in ['forward_41','backward_41'],command=task);o,_=w.reset(seed=seed);e=w.e
 if name=='backward_41':actor.legacy.reset(e)
 for k in range(w.horizon):
  o,_,t,tr,i=w.step(actor.predict(e,o))
  if frame:frame(w,i)
  if t or tr:break
 if task=='right':i['success']=bool(i['first_roll'] and i['net_turns']>=.20);i['two_stages']=bool(i['net_turns']>=.5 and not i['stood'] and i['valid'])
 i=dict(i,name=name,seed=seed,task=task);w.close();return i
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--name',required=True);p.add_argument('--n',type=int,default=20);p.add_argument('--seedbase',type=int,default=84000000);p.add_argument('--split',default='validation');a=p.parse_args();out=P/a.split/a.name;out.mkdir(parents=True,exist_ok=True);actor=Actor412(a.name);rows=[]
 for k in range(a.n):
  rows.append(episode(actor,a.seedbase+k));(out/'progress.json').write_text(json.dumps(dict(done=len(rows),total=a.n)))
 summary=dict(name=a.name,n=len(rows),success=float(np.mean([r['success'] for r in rows])),stood=float(np.mean([r['stood'] for r in rows])),invalid=sum(not r['valid'] for r in rows))
 if a.name.startswith('right'):summary.update(mean_turns=float(np.mean([r['net_turns'] for r in rows])),two_stages=float(np.mean([r['two_stages'] for r in rows])))
 else:summary.update(mean_forward_m=float(np.mean([r['forward_m'] for r in rows])),mean_late_forward_m=float(np.mean([r['late_forward_m'] for r in rows])),mean_lower_span_deg=float(np.mean([r['lower_span_deg'] for r in rows])),mean_upper_span_deg=float(np.mean([r['upper_span_deg'] for r in rows])))
 (out/'episodes.json').write_text(json.dumps(rows,indent=2));(out/'summary.json').write_text(json.dumps(summary,indent=2));print(summary,flush=True)
