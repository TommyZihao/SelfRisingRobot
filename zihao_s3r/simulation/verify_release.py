"""Check release hashes and the compiled MJCF's hardware interface."""
import hashlib,json
from pathlib import Path
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1]
m=json.loads((ROOT/'manifest.json').read_text())
for key,path in [('selected_model_sha256',ROOT/'models/robo_s3r_v3.zip'),('header_sha256',ROOT/'firmware/robo03_atom_s3r_zihao_v3/policy_network.h'),('xml_sha256',ROOT/'simulation/robo_s3r.xml')]:
    actual=hashlib.sha256(path.read_bytes()).hexdigest()
    if actual!=m[key]:raise RuntimeError(f'Hash mismatch: {path.name}')
model=mujoco.MjModel.from_xml_path(str(ROOT/'simulation/robo_s3r.xml'))
for name,angle in [('servo1',np.pi/2),('servo2',2*np.pi/3)]:
    np.testing.assert_allclose(model.joint(name+'_joint').range,[-angle,angle],atol=1e-10)
    np.testing.assert_allclose(model.actuator(name+'_pos').ctrlrange,[-angle,angle],atol=1e-10)
np.testing.assert_allclose(model.body_mass.sum(),.103694,atol=1e-10)
print('Verified release hashes, lower/upper limits and nominal mass.')
