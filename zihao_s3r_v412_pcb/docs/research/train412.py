# 作者：同济子豪兄 2026-10-26
import argparse,json,time
from pathlib import Path
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.logger import configure
from env412 import RightEnv,ForwardEnv
p=argparse.ArgumentParser();p.add_argument('--task',choices=['right','forward','backward'],required=True);p.add_argument('--init',choices=['scratch','warm','mirror'],required=True);p.add_argument('--seed',type=int,required=True);p.add_argument('--steps',type=int,default=800000);a=p.parse_args();P=Path(__file__).resolve().parent;torch.set_num_threads(1)
name=f'{a.task}_{a.init}_{a.seed}';out=P/'runs'/name;out.mkdir(parents=True,exist_ok=True);(out/'config.json').write_text(json.dumps(dict(author='同济子豪兄 2026-10-26',**vars(a)),indent=2))
e=DummyVecEnv([lambda i=i:Monitor(RightEnv() if a.task=='right' else ForwardEnv(command=a.task),str(out/f'env{i}')) for i in range(4)])
m=PPO('MlpPolicy',e,seed=a.seed,device='cpu',learning_rate=3e-4,n_steps=512,batch_size=256,n_epochs=5,ent_coef=.025,gamma=.995,target_kl=.03,policy_kwargs=dict(net_arch=[64,64],activation_fn=torch.nn.Tanh))
if a.init!='scratch':
 old=PPO.load(P.parent/'walk_continuous_roll/runs/warm_1101/final.zip' if a.task=='right' else P.parent/'walk_forward_down/runs/scratch_702/final.zip',device='cpu');m.policy.load_state_dict(old.policy.state_dict())
 if a.init=='mirror':
  with torch.no_grad():
   m.policy.mlp_extractor.policy_net[0].weight[:,[1,3,5,7,9,10,12,14,16]]*=-1
   m.policy.action_net.weight[0]*=-1;m.policy.action_net.bias[0]*=-1
 for layer in m.policy.mlp_extractor.value_net:
  if hasattr(layer,'reset_parameters'):layer.reset_parameters()
 m.policy.value_net.reset_parameters()
# Restore per-run RNG after loading a pretrained checkpoint.
m.set_random_seed(a.seed);m.set_logger(configure(str(out),['csv']));start=time.time();m.learn(a.steps,log_interval=5);m.save(out/'final');e.close();(out/'complete.json').write_text(json.dumps(dict(steps=m.num_timesteps,seconds=time.time()-start)));print(name,'DONE',flush=True)
