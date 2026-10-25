"""Repeat the frozen 2,000-episode held-out evaluation (no firmware state machine)."""
import argparse,csv
from pathlib import Path
import torch
from stable_baselines3 import PPO
from s3r_training_env import S3RTrainingEnv,POSES,SCENARIOS
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--model',type=Path,default=ROOT/'models/robo_s3r_v3.zip');p.add_argument('--n',type=int,default=100);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    torch.set_num_threads(1);m=PPO.load(a.model,device='cpu');e=S3RTrainingEnv(True,True);rows=[]
    for ki,k in enumerate(SCENARIOS):
        for pi,pose in enumerate(POSES):
            for i in range(a.n):
                seed=4000000+ki*100000+pi*10000+i;o,info=e.reset(seed=seed,options={'scenario':k,'pose':pose})
                for step in range(700):
                    if info['invalid']:break
                    o,_,done,truncated,info=e.step(m.predict(o,deterministic=True)[0])
                    if done or truncated:break
                rows.append({'seed':seed,'scenario':k,'pose':pose,'success':int(info['is_success']), 'invalid':int(info['invalid']),'ever_fallen':int(info['ever_fallen']),'steps':step+1})
            print(k,pose,sum(r['success'] for r in rows[-a.n:]),'/',a.n,flush=True)
    with a.out.open('x') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    print('Total',sum(r['success'] for r in rows),'/',len(rows));e.close()
if __name__=='__main__':main()
