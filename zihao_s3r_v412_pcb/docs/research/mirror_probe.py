# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import sys,json
import numpy as np,torch
from stable_baselines3 import PPO
P=Path(__file__).resolve().parent;sys.path.insert(0,str(P.parent/'walk_continuous_roll'))
from continuous_env import ContinuousRoll
torch.set_num_threads(1)
m=PPO.load(P.parent/'walk_continuous_roll/runs/warm_1101/final.zip',device='cpu')
mask=np.ones(22,dtype='float32');mask[[1,3,5,7,9,10,12,14,16]]=-1
rows=[]
for kind in ['original','mirrored']:
 for seed in range(81000000,81000012):
  e=ContinuousRoll(horizon=600);o,_=e.reset(seed=seed,options={'command':'roll_right'})
  peak=0;stable=0;passed=False
  for k in range(600):
   a=m.predict(o*mask if kind=='mirrored' else o,deterministic=True)[0]
   if kind=='mirrored':a[0]*=-1
   o,_,t,tr,i=e.step(a);peak=max(peak,i['net_turns']);stable=stable+1 if i['net_turns']>=.24 else 0;passed|=stable>=10
   if t or tr:break
  rows.append(dict(kind=kind,seed=seed,first_roll=bool(passed and not i['stood']),peak_turns=peak,**i));e.close()
 for k in [kind]:
  g=[r for r in rows if r['kind']==k];print(kind,'first',sum(r['first_roll'] for r in g),'mean',np.mean([r['net_turns'] for r in g]),flush=True)
(P/'mirror_probe.json').write_text(json.dumps(rows,indent=2))
