"""S3R recovery benchmark. Observation/action order: lower GPIO1, upper GPIO2.

Surface/friction and actuator uncertainties are provisional engineering ranges,
not measurements of the user's floor. No firmware mode switching is simulated.
"""
from pathlib import Path
import numpy as np
import mujoco
from gymnasium import spaces
from robo1_env import Robo1GetupEnv, FALLEN_POSES, roll_pitch_to_quat

XML = str(Path(__file__).with_name('robo_s3r.xml'))
POSES = tuple(FALLEN_POSES)
SCENARIOS = ('fixed', 'lying', 'kick', 'oblique', 'inverted')


class S3RTrainingEnv(Robo1GetupEnv):
    def __init__(self, diverse=True, randomize=True):
        super().__init__(XML)
        self.diverse, self.randomize = diverse, randomize
        self.target_limit = np.array([np.pi / 2, 2 * np.pi / 3])
        bounds = np.r_[np.pi, np.pi / 2, self.target_limit].astype(np.float32)
        self.observation_space = spaces.Box(-bounds, bounds)
        self.nominal = {n: getattr(self.model, n).copy() for n in
                        ('body_mass', 'body_inertia', 'geom_friction',
                         'dof_damping', 'actuator_gainprm', 'actuator_biasprm')}
        self.noise = 0.
        self.delay = 0
        self.queue = []
        self.previous_action = np.zeros(2)
        self.scenario = 'fixed'
        self.ever_fallen = False

    def physical_randomization(self):
        for n, v in self.nominal.items():
            getattr(self.model, n)[:] = v
        self.noise, self.delay = 0., 0
        if self.randomize:
            rng = self.np_random
            scales = rng.uniform(.9, 1.1, self.model.nbody)
            self.model.body_mass[:] *= scales
            self.model.body_inertia[:] *= scales[:, None]
            # Dry hard surfaces, broad provisional range; not identified wood coefficients.
            self.model.geom_friction[:, 0] = rng.uniform(.35, 1.0)
            self.model.dof_damping[:] *= rng.uniform(.7, 1.3)
            strength = rng.uniform(.75, 1.25)
            self.model.actuator_gainprm[:, 0] *= strength
            self.model.actuator_biasprm[:, 1:3] *= strength
            self.noise = rng.uniform(0, .015)
            self.delay = int(rng.integers(0, 3))
        mujoco.mj_setConst(self.model, self.data)

    def observed(self):
        o = self._get_obs()
        if self.noise:
            o[:2] += self.np_random.normal(0, self.noise, 2)
            o[:2] = np.clip(o[:2], self.observation_space.low[:2], self.observation_space.high[:2])
        return o

    def valid(self):
        return bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all()
                    and not any(w.number for w in self.data.warning))

    def reset(self, seed=None, options=None):
        # Use Gym's seed handling without the old fixed-pose reset.
        super(Robo1GetupEnv, self).reset(seed=seed)
        options = options or {}
        rng = self.np_random
        self.physical_randomization()
        mujoco.mj_resetData(self.model, self.data)
        self.scenario = options.get('scenario', rng.choice(SCENARIOS, p=[.15,.35,.25,.15,.10]) if self.diverse else 'fixed')
        self.current_pose_name = options.get('pose', rng.choice(POSES))
        roll, pitch = FALLEN_POSES[self.current_pose_name]
        self.data.qpos[:3] = [0, 0, .08]
        self.target[:] = 0
        if self.scenario == 'kick':
            roll = pitch = 0.
        elif self.scenario != 'fixed':
            if self.scenario == 'inverted':
                roll, pitch = np.pi + rng.uniform(-.5, .5), rng.uniform(-.5, .5)
            else:
                spread = options.get("tilt_spread", .45)
                roll += rng.uniform(-spread, spread)
                pitch += rng.uniform(-spread, spread)
            if rng.random() < .65:
                self.target[:] = rng.uniform(-.6, .6, 2) * self.target_limit
        q = roll_pitch_to_quat(roll, pitch)
        yaw = rng.uniform(-np.pi, np.pi) if self.scenario != 'fixed' else 0.
        qy = np.array([np.cos(yaw/2), 0, 0, np.sin(yaw/2)])
        mujoco.mju_mulQuat(self.data.qpos[3:7], qy, q)
        self.data.qpos[[self.servo1_qpos_id, self.servo2_qpos_id]] = self.target
        self.data.ctrl[:] = self.target
        mujoco.mj_forward(self.model, self.data)
        # Place the lowest collision vertex just above the floor before settling.
        lowest = np.inf
        for gid in range(self.model.ngeom):
            if self.model.geom_bodyid[gid] == 0 or not self.model.geom_contype[gid]:
                continue
            typ = self.model.geom_type[gid]
            if typ == mujoco.mjtGeom.mjGEOM_MESH:
                mid = self.model.geom_dataid[gid]
                adr, num = self.model.mesh_vertadr[mid], self.model.mesh_vertnum[mid]
                verts = self.model.mesh_vert[adr:adr+num]
            elif typ == mujoco.mjtGeom.mjGEOM_BOX:
                verts = np.array([[a,b,c] for a in (-1,1) for b in (-1,1) for c in (-1,1)]) * self.model.geom_size[gid]
            else:
                continue
            z = verts @ self.data.geom_xmat[gid].reshape(3,3)[2] + self.data.geom_xpos[gid,2]
            lowest = min(lowest, float(z.min()))
        self.data.qpos[2] += .003 - lowest
        mujoco.mj_forward(self.model, self.data)
        settle = 200 if self.scenario == 'fixed' else (int(rng.integers(80, 250)) if self.scenario == 'oblique' else 800)
        for _ in range(settle):
            mujoco.mj_step(self.model, self.data)
        if self.scenario == 'kick':
            # Push the upper assembly horizontally in a direction tied to the pose label.
            directions = {'roll_pos': [0,-1], 'roll_neg': [0,1], 'pitch_pos': [1,0], 'pitch_neg': [-1,0]}
            direction = np.array(directions[self.current_pose_name], dtype=float)
            spread = options.get("kick_spread", .3)
            angle = yaw + rng.uniform(-spread, spread)
            direction = np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]) @ direction
            body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'arm2')
            self.data.xfrc_applied[body,:2] = direction * rng.uniform(.5, 2.5)
            for _ in range(int(rng.integers(50, 181))):
                mujoco.mj_step(self.model, self.data)
            self.data.xfrc_applied[:] = 0
            for _ in range(int(rng.integers(0, 151))):
                mujoco.mj_step(self.model, self.data)
        self.step_count = self.success_count = 0
        self.previous_action[:] = 0
        self.prev_upright = self._upright()
        self.ever_fallen = self.prev_upright < .8
        self.queue = [self.target.copy() for _ in range(self.delay)]
        return self.observed(), self.info(False)

    def info(self, success):
        return {'is_success': bool(success), 'pose': self.current_pose_name,
                'scenario': self.scenario, 'ever_fallen': self.ever_fallen,
                'invalid': not self.valid(), 'upright': self._upright(),
                'target': self.target.copy()}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=float), -1, 1)
        self.target[:] = np.clip(self.target + .08 * action, -self.target_limit, self.target_limit)
        self.queue.append(self.target.copy())
        applied = self.queue.pop(0)
        self.data.ctrl[:] = applied
        for _ in range(20):
            mujoco.mj_step(self.model, self.data)
        self.step_count += 1
        if not self.valid():
            return np.zeros(4, dtype=np.float32), -100., True, False, self.info(False)
        upright = self._upright()
        self.ever_fallen |= upright < .8
        o = self._get_obs()
        joints = self._servo_angles()
        speed = np.linalg.norm(self.data.qvel[:6])
        goal = (upright > .95 and np.max(np.abs(o[:2])) < .25
                and np.max(np.abs(joints)) < .15 and np.max(np.abs(self.target)) < .18
                and speed < .3 and abs(self._foot_height()-self.standing_height) < .02)
        self.success_count = self.success_count + 1 if goal else 0
        success = self.success_count >= 50
        gate = np.clip((upright-.65)/.35, 0, 1)
        reward = (1.5*upright + 8*(upright-self.prev_upright) - 1.
                  - gate*.4*np.sum(joints*joints) - .002*np.sum(action*action)
                  - .002*min(speed*speed, 100) + 5*goal + 100*success)
        self.prev_upright = upright
        self.previous_action[:] = action
        return self.observed(), float(reward), success, self.step_count >= 700, self.info(success)
