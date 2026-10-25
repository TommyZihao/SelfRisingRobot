"""Run the bundled V3 actor on the S3R model; continue controlling after standing."""
import argparse
from pathlib import Path
import time
import numpy as np
import mujoco
import torch
from stable_baselines3 import PPO
from s3r_training_env import S3RTrainingEnv, POSES

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=Path, default=ROOT/'models/robo_s3r_v3.zip')
    parser.add_argument('--pose', choices=POSES, default='pitch_pos')
    parser.add_argument('--seed', type=int, default=11)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--steps', type=int, default=1500, help='headless duration at 50 Hz')
    args = parser.parse_args()
    torch.set_num_threads(1)
    env = S3RTrainingEnv(diverse=False, randomize=False)
    model = PPO.load(args.model, device='cpu')
    obs, _ = env.reset(seed=args.seed, options={'pose': args.pose})
    if args.headless:
        first_success = None
        for i in range(args.steps):
            obs, _, _, _, info = env.step(model.predict(obs, deterministic=True)[0])
            if info['invalid']:
                raise RuntimeError('MuJoCo numerical instability')
            if info['is_success'] and first_success is None:
                first_success = (i+1)*.02
        print({'pose': args.pose, 'first_success_seconds': first_success,
               'final_roll_pitch_degrees': np.rad2deg(obs[:2]).tolist()})
        env.close()
        return
    import mujoco.viewer
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        viewer.cam.lookat[:] = env.data.subtree_com[env.foot_body_id]
        viewer.cam.distance = .45
        while viewer.is_running():
            start = time.monotonic()
            obs, _, _, _, info = env.step(model.predict(obs, deterministic=True)[0])
            if info['invalid']:
                raise RuntimeError('MuJoCo numerical instability; restart the viewer')
            viewer.sync()
            time.sleep(max(0, .02-(time.monotonic()-start)))
    env.close()

if __name__ == '__main__':
    main()
