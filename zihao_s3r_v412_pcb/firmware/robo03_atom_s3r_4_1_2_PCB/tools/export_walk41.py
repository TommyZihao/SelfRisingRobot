# 作者：同济子豪兄 2026-10-26
import argparse,json
from pathlib import Path
import torch
from stable_baselines3 import PPO
p=argparse.ArgumentParser();p.add_argument('--models',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
for filename,ns in [('legacy_walk.zip','walk41_legacy'),('continuous_roll.zip','walk41_roll'),('right_roll.zip','walk412_right')]+([('forward_ppo.zip','walk412_forward')] if (a.models/'forward_ppo.zip').exists() else [])+([('backward_ppo.zip','walk412_backward')] if (a.models/'backward_ppo.zip').exists() else []):
 m=PPO.load(a.models/filename,device='cpu');layers=[x for x in m.policy.mlp_extractor.policy_net if isinstance(x,torch.nn.Linear)]+[m.policy.action_net]
 text=['// 作者：同济子豪兄 2026-10-26 | RoBoBo 4.1.2-PCB','#pragma once','#include <math.h>',f'namespace {ns} {{']
 for i,l in enumerate(layers):
  w=l.weight.detach().numpy();b=l.bias.detach().numpy()
  fmt=lambda x:format(float(x),'.9e')+'f'
  text += [f'static const float w{i}[{w.shape[0]}][{w.shape[1]}]={{'+','.join('{'+','.join(map(fmt,row))+'}' for row in w)+'};',f'static const float b{i}[{len(b)}]={{'+','.join(map(fmt,b))+'};']
 text+=['inline void forward(const float* input,float* out){','const float* current=input;']
 for i,l in enumerate(layers):
  n,k=l.weight.shape;text+=[f'float h{i}[{n}];',f'for(int r=0;r<{n};++r){{float sum=b{i}[r];for(int c=0;c<{k};++c)sum+=w{i}[r][c]*current[c];h{i}[r]='+('tanhf(sum)' if i<len(layers)-1 else 'sum')+';}',f'current=h{i};']
 text+=['for(int i=0;i<2;++i)out[i]=fmaxf(-1.f,fminf(1.f,current[i]));','}','}']
 (a.output/(ns+'.h')).write_text('\n'.join(text)+'\n')
