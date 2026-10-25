# Training and evaluation provenance

## Selected actor

`ppo_scratch_long_seed11/best.zip` is distributed as `models/robo_s3r_v3.zip`. It was initialized randomly, with no demonstrations or V2.1 actor transfer. The lineage first ran about 400,000 PPO steps, then continued for about 1,600,000 additional steps. The selected checkpoint is around 1,800,000 cumulative steps, not the final checkpoint. `manifest.json` records its SHA-256 and the exported header hash.

Architecture and PPO settings: 4 inputs, two 64-unit tanh hidden layers, 2 linear outputs; 4 vectorized environments, rollout length 512 per environment, batch size 256, 5 epochs per update, learning rate 1e-4, gamma 0.995, entropy coefficient 0.01 and target KL 0.03. Control is 20 physics steps of 1 ms per action. The four inputs do not include angular velocity, so dynamically different states can have the same observation.

## Comparisons performed

Eight configurations each used seeds 11 and 29 and approximately 400,000 steps:

1. PPO scratch, fixed falls, nominal physics.
2. PPO scratch, diverse falls, nominal physics.
3. PPO scratch, diverse falls, randomized physics.
4. PPO demonstration initialization, diverse falls, nominal physics.
5. PPO demonstration initialization, diverse falls, randomized physics.
6. A2C scratch, diverse falls, randomized physics.
7. A2C demonstration initialization, diverse falls, randomized physics.
8. PPO initialized from the V2.1 actor, diverse falls, randomized physics.

Four additional runs: two V2.1-initialized low-exploration-noise PPO runs of 1 million steps each, and two scratch PPO continuations of 1.6 million steps each. These are unequal-budget experiments, not a controlled claim that one initialization or algorithm always wins. Two seeds also give limited evidence about training variability.

Demonstration collection tried 240 new-physics states: 204 succeeded with V2.1, 29 more with reference trajectories. Only successful actually-fallen trajectories were used, providing 31,538 observation/action samples. Those demonstrations did **not** initialize the released actor.

Only the released checkpoint is bundled. Historical candidate checkpoints, demonstration data and the V2.1 weights are not duplicated here. `comparison.csv` and the figures retain the broader results; the scratch training helper is not an orchestrator for reconstructing every historical experiment.

## Reset and physics distributions

Training mixture: 15% fixed four-direction falls, 35% settled lying states, 25% kicks, 15% partially settled oblique falls and 10% inverted states. Targets vary for many non-fixed resets. Random yaw is included. Mesh collision vertices are used to place the initial assembly above the floor before settling.

Kicks apply a 0.5–2.5 N horizontal force to the upper assembly for 50–180 ms, with direction jitter and a variable post-release delay. Servo commands are neutral during the pulse; recovery starts afterward. This differs from arbitrary impulses applied while the policy is already executing. Evaluation also records whether a trial actually became fallen.

Randomization ranges: masses/inertias 0.9–1.1×, sliding friction 0.35–1.0, joint damping 0.7–1.3×, actuator gains 0.75–1.25×, angle noise standard deviation 0–0.015 rad and control delay 0–40 ms. These are provisional hard-surface ranges, not measured coefficients for a particular wooden floor or desk.

## Success and dataset splits

A success requires 50 consecutive control steps (one second) with:

- base upright component >0.95 and absolute roll/pitch <0.25 rad;
- absolute simulated joint angles <0.15 rad and targets <0.18 rad;
- root velocity norm <0.3 and base-origin height within 0.02 m of the standing reference.

Timeout is 700 control steps (14 seconds). Any MuJoCo numerical warning counts as failure. This is an episode-evaluation horizon, **not** a V3.1 firmware stop condition.

Five scenarios × four direction strata are evaluated. Inverted labels identify sampling strata, not four distinct inversion mechanisms. Seed formula is `start + scenario_index*100000 + pose_index*10000 + trial_index`, with ordering defined in `s3r_training_env.py`.

- Repeated screening: start 1,000,000; 2 trials per cell (40 total), used to select checkpoints.
- Validation: start 2,000,000; 20 trials per cell (400 per candidate), used to rank candidates.
- Selection score: 70% overall success + 30% weakest direction-stratum success.
- Frozen holdout: start 4,000,000; 100 trials per cell (2,000 per actor), evaluated after selection. No subsequent policy tuning used these outcomes.
- Additional stress: start 8,000,000; 300 trials per actor, kick jitter ±45° and lying/oblique offsets ±60°. Not used for selection.

Release holdout: 1968/2000 vs V2.1's 1701/2000. The paired improvement is 13.35 percentage points, with a bootstrap 95% interval of [11.65, 15.00] points. The individual confidence intervals are Wilson intervals. Both policies use the new physics and limits for this comparison; this is not a direct measurement of the old deployed firmware.

## Train a new scratch candidate

From the repository root after installing dependencies:

```sh
zihao_s3r/.venv/bin/python zihao_s3r/simulation/train_scratch.py \
  --seed 11 --steps 400000 --out zihao_s3r/runs/scratch_400k
zihao_s3r/.venv/bin/python zihao_s3r/simulation/train_scratch.py \
  --resume zihao_s3r/runs/scratch_400k/final.zip \
  --steps 1600000 --out zihao_s3r/runs/scratch_extended
```

The helper uses the selected experiment's PPO parameters and saves every 100,000 interaction steps. It does not automatically choose the best checkpoint. Compare on a separate validation set before exporting. Because library/hardware/runtime changes can alter learning, do not expect byte-identical weights even with the same seed. `evaluation/requirements.lock.txt` records the original environment.

To export a new PPO actor, explicitly choose a destination outside the release header until it has been verified:

```sh
zihao_s3r/.venv/bin/python zihao_s3r/simulation/export_policy_header.py \
  PATH_TO_NEW_MODEL.zip --output NEW_policy_network.h
```

## Logs and plots

`evaluation/scratch_first_400k_*` and `scratch_additional_1600k_*` contain the winning lineage's raw training CSV and repeated screening evaluations. Extended-run step counts restart at zero. Loss definitions differ across algorithms; low loss does not imply high recovery success. The PNGs include broader comparison curves, held-out rates and representative successful/failed trajectories. No claim is made that the remaining roll_neg failures have been eliminated.
