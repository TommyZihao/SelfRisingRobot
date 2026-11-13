# 作者：同济子豪兄 2026-10-26
from pathlib import Path
import numpy as np,torch,json
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from env412 import RightEnv,ForwardEnv
P=Path(__file__).resolve().parent;torch.set_num_threads(1)
for C in [RightEnv,ForwardEnv]:
 e=C();check_env(e);o,_=e.reset(seed=42);assert e.e.actual_pose=='pitch_neg';assert o.shape==(22,);e.close()
m=PPO.load(P.parent/'walk_continuous_roll/runs/warm_1101/final.zip',device='cpu');x=np.random.default_rng(8).uniform(-3,3,(500,22)).astype('float32');mask=np.ones(22,dtype='float32');mask[[1,3,5,7,9,10,12,14,16]]=-1
expected=m.predict(x*mask,deterministic=True)[0];expected[:,0]*=-1
with torch.no_grad():
 m.policy.mlp_extractor.policy_net[0].weight[:,[1,3,5,7,9,10,12,14,16]]*=-1;m.policy.action_net.weight[0]*=-1;m.policy.action_net.bias[0]*=-1
assert np.max(abs(expected-m.predict(x,deterministic=True)[0]))<1e-6
out=P/'runs/right_mirror_initial';out.mkdir(parents=True,exist_ok=True);m.save(out/'final');(out/'config.json').write_text(json.dumps(dict(task='right',init='mirror_initial',steps=0,author='同济子豪兄 2026-10-26')))
e=ForwardEnv();e.reset(seed=43);old=e.e.target.copy();e.step(np.ones(2));assert np.all(abs(e.e.target-old)<=np.array([.06,.08])+1e-8);e.close()
(P/'checks.json').write_text(json.dumps(dict(env_checker=True,mirror_export_exact=True,forward_slew_limits=True),indent=2));print('ENV/MIRROR/SLEW CHECKS PASSED')
