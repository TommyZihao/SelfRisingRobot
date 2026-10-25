"""PPO from scratch with the selected experiment's architecture and parameters.

The released actor was selected from checkpoints, not simply the final weights.
This helper trains new weights without overwriting the release.
"""
import argparse
from pathlib import Path
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure
from s3r_training_env import S3RTrainingEnv

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--seed',type=int,default=11)
    p.add_argument('--steps',type=int,default=400000)
    p.add_argument('--resume',type=Path)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(1)
    env=DummyVecEnv([lambda i=i:Monitor(S3RTrainingEnv(True,True),str(a.out/f'env{i}')) for i in range(4)])
    if a.resume:
        model=PPO.load(a.resume,env=env,device='cpu')
    else:
        model=PPO('MlpPolicy',env,seed=a.seed,device='cpu',gamma=.995,
                  learning_rate=1e-4,ent_coef=.01,n_steps=512,batch_size=256,
                  n_epochs=5,target_kl=.03,
                  policy_kwargs={'net_arch':dict(pi=[64,64],vf=[64,64]),'activation_fn':torch.nn.Tanh})
    model.set_logger(configure(str(a.out),['stdout','csv']))
    model.learn(total_timesteps=a.steps,log_interval=10,
                callback=CheckpointCallback(save_freq=25000,save_path=str(a.out),name_prefix='checkpoint'))
    model.save(a.out/'final');env.close()

if __name__ == '__main__':main()
