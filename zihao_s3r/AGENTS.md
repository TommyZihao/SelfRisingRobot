# Instructions for AI contributors

This directory is the ATOM S3R V3.1 variant. Read README.md and docs/TRAINING.md before changing the controller or policy. Keep changes scoped here unless the user requests upstream-wide changes.

## Interface invariants

- Observations: `[roll, pitch, lower_target, upper_target]`, radians, no normalization. Targets are commanded positions, not measured angles.
- Actions: clipped to [-1,1], integrated by 0.08 rad at approximately 50 Hz.
- Lower = `servo1_joint` = action 0 = GPIO1 = ±π/2.
- Upper = `servo2_joint` = action 1 = GPIO2 = ±2π/3.
- Firmware angle-to-pulse mapping is negative: positive angle decreases pulse width. Do not reverse signs or channels based solely on names in legacy variants.
- The released actor is a 4→64→64→2 tanh/tanh/linear network. Export only the actor, not the critic.

## Control behavior

V3.1 continuously runs the actor in automatic mode. DONE is display-only. Do not reintroduce the 43° gate, 300 ms delay, automatic timeout, or automatic hard return to center. Pause/off must hold targets; resume must preserve them. Explicit manual control is a separate user action and must synchronize stored targets. Do not silently change weights when fixing firmware behavior.

## Physics and evidence

Use simulation/robo_s3r.xml, not the root project's original XML. Preserve the battery and asymmetric joint/actuator limits. The controller geometry, servo dynamics, friction and PWM calibration are approximations; do not describe them as measured. Do not claim 98.4% real-world success. The owner reported good real-world behavior, but only the supplied simulation evaluation has a measured rate.

The held-out seed range beginning at 4,000,000 was used to report the release result. Once used to tune a successor, it is no longer an untouched holdout. Choose new seeds for final tests. Numerical warnings are failures, not successful resets. Keep trial denominators, scenario definitions and actual-fall subsets visible.

## Validation and release edits

- Load the MJCF and check compiled joint and actuator ranges after physics changes.
- Run playback headlessly from a different working directory to check relative-path independence.
- Build with `m5stack:esp32:m5stack_atoms3` and the dependency versions in README.md.
- After policy export, compare raw C inference against PyTorch over representative observations and update manifest hashes. Previously measured max error was about 1.91e-6 over 2,000 observations.
- Keep model weights, exported header and manifest aligned. Do not commit local serial identities, personal absolute paths, build caches, `.elf`/`.map`, or credentials.
- Do not overwrite the released model during training. The training helper requires a new output directory; preserve raw logs and explicit selection criteria.

Firmware upload moves real hardware. Only upload when the user authorizes hardware deployment; ordinary code-review requests do not imply it.
