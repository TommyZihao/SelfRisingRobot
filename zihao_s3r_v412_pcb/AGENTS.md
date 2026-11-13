# RoBoBo 4.1.2-PCB maintenance

Author: 同济子豪兄 2026-10-26. Read README.md, release.json, models/provenance.json and validation.json first. Original project credit: HomeMadeGarbage / SelfRisingRobot.

Preserve V3.1 recovery/IMU mapping, four modes, GPIO1+39 lower and GPIO2+38 upper, stored PWM calibration, 200Hz opt-in telemetry, heartbeat retry, active-only400ms watchdog and home-to-Walk-idle behavior. Targets are not measured joint angles. Do not claim all-pose locomotion, reliable continuous roll, yaw improvement or real-device validation.

Forward and backward: selected CEM sine targets plus heading feedback, not PPO. Config headers carry the full search parameters. Backward heading correction maintains the opposite of its displacement goal. Limits lower ±60°, upper ±120°, target slew .06/.08 rad per20ms. Left roll uses old warm_1101; right roll uses right_mirror_2101. Both use22D delta PPO and continuous gravity-azimuth stages. Yaw keeps16D legacy PPO+gaits. Never interchange observation layouts. No actual joint position, world position or contact sensor in actor inputs.

Keep original weights, provenance, scalar parameters, parity tests and report. Same-command heartbeats preserve phase; release and reversal preserve physical state. Update source, compiled binaries, manifest and hashes together. This version was flashed and published with user authorization on 2026-11-13. Flash hashes and serial responses passed; motion performance is not yet verified.

Exclude environments, caches, ELF/map, credentials and workstation paths. Full simulation tests require original research RL directory; network parity and viewer checks run standalone with dependencies. Report first-quarter roll success separately from continuous roll; report forward/backward sustained movement criteria rather than distance alone.
