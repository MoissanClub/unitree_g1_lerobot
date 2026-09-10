# Rung 4 Handoff

## Resume Point

Resume using the [three-track project plan](project-plan.md): Track 1 owns G1-23
LeRobot support (`robots/`), Track 2 owns XR (`xr/`), and Track 3 owns G1 simulation
(`simulation/`). Track 3's native supported-arm G1-23 backend now runs, including
basic standalone/embedded DDS checks. Next is systematic Track 1 joint/IK/DDS
acceptance; Track 2 headset acceptance follows. Existing rung names remain shared
acceptance gates, not separate implementation tracks.

The configurable runtime structure has now been implemented and tested after the
benchmark review. See [architecture](architecture.md#configurable-g1-runtime-structure).
Use `unitree_g1_lerobot.robots.unitree_g1` to register G1-23 and import the shared
LeRobot classes. The LeRobot checkout changes are reproduced by
`apply_lerobot_embodiment_patch.sh` and the versioned patch; there are no variant robot
subclasses. Configuration, active features, sparse DDS mapping, motor defaults, and
IK/gravity selection are implemented. G1-23 `connect()` now creates the native
supported-arm backend; hardware remains blocked. See [live simulator](g1-23-live-simulator.md)
for the tested standalone launcher, confirmation flow, and remaining acceptance boundary.

The user has confirmed visual review of the motor-comparison checkpoint. Geometry,
scripted IK, and the supported-arm motor benchmarks are reviewed. **Rung 4 is not
complete:** next work is systematic per-joint, Cartesian/DDS, and XR acceptance.
Do not repeat the completed comparisons or backend bring-up as a new milestone.

Last pushed benchmark baseline: `94ba97f` on `main`, pushed to
`git@github.com:MoissanClub/unitree_g1_lerobot.git`. The structural refactor and these
handoff updates are subsequent work; check both repository worktrees before resuming.
Project directory:
`~/lerobot-sim/unitree_g1_lerobot`; adjacent LeRobot checkout: `../lerobot`.
Python environment: `~/miniforge3/envs/lerobot-g1/bin/python`.

## Completed Artifacts

- Preserve `run_compare_g1_29_g1_23_urdf_mesh.sh` and
  `run_compare_g1_29_g1_23_ik.sh` as independent geometry and kinematics checks.
- Source-derived profiles: `configs/motors/g1_29.json` and `g1_23.json`.
  Derivation and pinned upstream provenance live in
  `unitree_g1_lerobot/robots/motor_configs.py` and `assets/g1/motor_sources/`.
- Physics benchmark: `unitree_g1_lerobot/simulation/motor_bench.py`.
- Shared 34-case suite: `unitree_g1_lerobot/diagnostics/motor_suite.py`.
- Reports and Tk replay: `unitree_g1_lerobot/diagnostics/compare_motor_configs.py`.

From an `ssh -Y` terminal, run one launcher at a time:

```bash
cd ~/lerobot-sim/unitree_g1_lerobot
./run_compare_g1_29_motor_configs_no_gravity_compensation.sh
./run_compare_g1_29_motor_configs_with_gravity_compensation.sh
./run_compare_g1_29_g1_23_motor_configs_no_gravity_compensation.sh
./run_compare_g1_29_g1_23_motor_configs_with_gravity_compensation.sh
./compare_motor_config.sh
```

The summary columns are LeRobot G1-29, derived G1-29, and derived G1-23; rows are
compensation OFF above and ON below. All panels replay the same test time.
All five launchers compute physics and reports **before** opening the Tk viewer.
The user reported a missing summary window before later confirming review. An
early-window/progress implementation was proposed but interrupted and never applied;
do not assume it exists. If revisited, distinguish startup computation delay from
an X-display failure. `--tests pitch_20deg_step extended_hold_1kg` shortens startup.

## What Was Verified

All four pair launchers completed 34 scenarios and opened their viewers. The summary
completed 204 trials. Reports had the expected distinct traces and 34 previews.
Sixteen automated tests passed with real Tk/EGL checks enabled, including navigation,
pause/resume, restart, panel ordering, and robot-pixel motion in every panel.

```bash
G1_TEST_VIEWER=1 ~/miniforge3/envs/lerobot-g1/bin/python -m unittest discover -s tests -v
```

Without `G1_TEST_VIEWER=1`, only the real-viewer interaction test is skipped.
Results normally go under ignored `artifacts/motors/`; do not commit generated runs.
See [benchmark details](motor-config-comparison.md) for metrics and test conditions.

## Interpretation and Limits

- All runs simulate gravity. Compensated runs add exact-model gravity torque at
  measured joint positions with zero velocity in a separate MuJoCo data object.
  Known payload mass is included; no Coriolis, acceleration, or friction feedforward.
  Total PD-plus-feedforward torque is clipped to motor limits.
- Derived G1-29 shoulder pitch/roll gains are Kp=80 versus LeRobot Kp=50, both Kd=3.
  Other arm gains match. No final runtime gain choice or universal winner was approved.
- Faster steps/sweeps, short point-to-point moves, and reversals expose response-time,
  overshoot, and saturation tradeoffs. These are not maximum safe speed ratings.
- Extended holds use 0, 0.25, 0.5, and 1 kg per hand. The final pose separates the
  hands and is collision-free on both models. A narrower earlier pose caused G1-23
  hand collisions and was corrected. These loads are not hardware payload ratings.
- Pelvis, legs, and waist are supported. Comparisons use deterministic joint targets,
  not live IK/DDS, and playback replays measured physics trajectories.
- Cross-embodiment common-joint targets do not imply identical Cartesian hand paths.
  The five-joint arm cannot independently satisfy all six Cartesian pose objectives.

## Remaining Rung 4 Work

1. Preserve the completed native G1-23 runtime integration and basic motor-driven
   motion/feedback checks. Startup and stale-command holding use exact-model gravity
   compensation; active DDS control uses the sender's gains and feedforward without
   extra compensation. See [live simulator scope and verification](g1-23-live-simulator.md).
   The G1-29 launcher/viewer regression passed with its startup diagnostic skipped;
   broader G1-29 motion and headset regression remains pending.
2. Exercise each of ten active arm joints through a separate DDS sender. Verify
   names, sparse transport indices, directions, ranges, unused slots, and feedback.
3. Run deterministic Cartesian/orientation targets through IK -> DDS -> actuators.
   Measure IK residuals separately from actuator tracking error. Define acceptance
   thresholds and verify control timing, command stop, and stale-command behavior.
4. Select G1-23 in the existing XR bridge and verify startup diagnostics, engagement,
   release, tracking loss/disconnect, and headset control. Retain G1-29 regression
   evidence and document G1-23 orientation limitations.

No mandatory new keyboard stage: scripted DDS tests isolate integration variables.
Preserve existing G1-29 keyboard, DDS, and XR launchers. Keep robot support, simulation,
diagnostics, and CloudXR responsibilities separate for possible LeRobot contribution.
The user prefers continued implementation with minimal intervention; ask only when
a real tradeoff or required external observation blocks progress.

Robot-camera delivery to the headset remains unimplemented/unverified **Rung 4V**,
after control acceptance and before physical robots. Physical G1-29 baseline is
Rung 5a, followed by G1-23 work. A connected headset showing "Running" is not evidence
of camera streaming. Refer to the [ladder](vr-teleop-g1-23-ladder.md) for acceptance.
