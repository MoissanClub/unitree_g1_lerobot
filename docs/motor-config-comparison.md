# Motor Configuration Comparison

Two launchers run actual MuJoCo motor-driven arm tests, save quantitative results, then
continuously replay the recorded physics trajectories in a side-by-side Tk/X viewer.
They do not publish DDS commands and need neither CloudXR nor a headset.

## Run

From an `ssh -Y` terminal:

```bash
cd ~/lerobot-sim/unitree_g1_lerobot
./run_compare_g1_29_motor_configs_no_gravity_compensation.sh
./run_compare_g1_29_g1_23_motor_configs_no_gravity_compensation.sh
```

Run one at a time for review. The first compares Unitree-derived G1-29 gains (left)
against the current installed LeRobot G1-29 gains (right), on identical physics models.
The second compares Unitree-derived G1-29 (left) and G1-23 (right) on their native models.
All body motor settings are recorded in the two profiles, but this benchmark tests arms
only: pelvis, legs, and waist are rigidly supported, so their holding gains are not ranked.

The ten tests run before the viewer opens. Playback repeats the suite until the window
is closed or Ctrl+C is pressed. Pause, restart, previous/next, test selection, and playback
speed are available. Yellow spheres mark the commanded hand positions; green sites mark
the actual end-effector locations. Both cameras default to the robot's left-front:

```bash
./run_compare_g1_29_motor_configs_no_gravity_compensation.sh --camera-azimuth -135 --camera-elevation -10 --camera-distance 2.2
```

Bounded and headless checks:

```bash
./run_compare_g1_29_motor_configs_no_gravity_compensation.sh --no-view --output-dir /tmp/g1_motor_ab
./run_compare_g1_29_g1_23_motor_configs_no_gravity_compensation.sh --gui-seconds 10
./run_compare_g1_29_motor_configs_no_gravity_compensation.sh --tests shoulder_pitch_step shoulder_roll_step
```

Both launchers use `G1_PYTHON_BIN` when set, otherwise
`$HOME/miniforge3/envs/lerobot-g1/bin/python`. This environment also needs Matplotlib:

```bash
conda run -n lerobot-g1 python -m pip install 'matplotlib>=3.8,<3.11'
```

Matplotlib 3.10.9 was used for the initial verification. Existing MuJoCo, Pillow,
Hugging Face Hub, NumPy, and LeRobot dependencies come from the established environment.
An EGL-capable machine is needed for offscreen rendering, and Tk/X for the viewer.

## Method and Configuration Sources

`configs/motors/g1_29.json` and `g1_23.json` are generated with the same recipe:

1. Parse each pinned Unitree arm controller's gains, joint enums, wrist/weak-motor
   classifications, and control timestep. Apply its gain-selection rule by joint name.
2. Read actuator torque limits, joint ranges, damping, armature, and friction from the
   corresponding pinned Unitree MJCF. Retain sparse DDS indices; discard G1-23 placeholders.
3. Record source revisions and digests. At runtime, verify that the saved profiles still
   match their source derivation. The derivation does not tune or optimize the gains.

The G1-29 LeRobot baseline reads `UnitreeG1Config` from the active environment and records
its source file and digest in each report. It uses the identical Unitree physics model
as the derived G1-29. This is a comparison of gain choices, not a replication of every
detail of the existing Hub simulator or LeRobot transport implementation.

Validate or intentionally regenerate the profiles from the repository root:

```bash
python -m unitree_g1_lerobot.robots.motor_configs
python -m unitree_g1_lerobot.robots.motor_configs --write
```

See [source provenance and licenses](../assets/g1/motor_sources/README.md).

## Test Conditions

- 500 Hz MuJoCo physics and PD feedback; commanded joint targets update at 250 Hz.
- Torque control: `tau = clip(kp*(q_command-q) - kd*dq, +/-torque_limit)`.
- Gravity enabled; feedforward/gravity compensation disabled equally for both sides.
- Published native inertias, passive damping, armature, friction, limits, and collisions.
- Every test resets to the same supported ready pose and warms up for 0.75 seconds.
- Tests replay deterministic joint targets, so IK differences cannot alter the gain A/B.
- Across embodiments, the ten common arm joints get the same target angles. G1-29's extra
  wrist joints hold zero. The hand paths differ with geometry; this is not equal Cartesian
  workspace tracking or proof that one embodiment is better.

| Test | Purpose |
|---|---|
| Pose hold | Gravity bias and steady-state error |
| Shoulder pitch step | Gain-dependent transient response |
| Shoulder roll step | Gain-dependent transient response |
| Raise/lower | Shoulder pitch sweep |
| Forward/back | Elbow sweep |
| Left/right | Shoulder roll sweep |
| Wrist rotation | Common wrist-roll motion |
| Stop and hold | Residual excursion and motion after the target stops |
| Payload hold | Same 0.5 kg payload at each model's end-effector site, per hand |
| Delayed sweep | 20 ms target-delivery delay, with local PD feedback unchanged |

These tests cover representative arm behavior, not the complete joint workspace,
locomotion, electrical/thermal motor dynamics, or every hardware loading condition.

## Reports and Interpretation

Each run prints a unique output directory under ignored `artifacts/motors/` unless
`--output-dir` is supplied. It contains:

- `report.md`: readable results and measurement definitions.
- `report.json`: full metrics, conditions, source profiles, and library versions.
- `summary.csv`: per-test/profile metrics for comparison.
- `comparison.png`: overlaid actual/target joint motion, hand error, and torque demand.
- `preview_*.png`: side-by-side snapshots of each test.
- `*.npz`: full 500 Hz time series, including desired/delivered targets and measured motion.

Compare common-joint RMS error, hand pose error, lag, overshoot, settling, torque demand,
saturation, torque slew, and stopping motion. Common-joint RMS uses ten joints on both
models to avoid diluting G1-29 error across its extra four wrist joints.

Step overshoot is reported both relative to the commanded target and to the achieved
final equilibrium. The latter exposes oscillation even when gravity bias prevents the
joint from reaching the target. Settling bands are 0.02 rad around the target and 0.005 rad
around equilibrium; null means not settled within the test or not a step test.
Estimated lag is a bounded cross-correlation estimate, not measured DDS/network latency.

The runner fails on nonfinite physics, MuJoCo warnings, targets outside limits, joint-limit
violation above 0.03 rad, or speed above 20 rad/s. These are simulation-integrity guardrails,
not final hardware acceptance thresholds. It deliberately does not declare an automatic
winner: better tracking may come with more overshoot and stiffer response.

The first G1-29 tests showed this tradeoff: the derived gains reduced tracking error,
but step overshoot around equilibrium was higher than with LeRobot's defaults. Inspect
the reports for the actual run rather than assuming higher gains are universally better.

This completes a supported-arm physics comparison artifact. Rung 4 still needs the live
G1-23 LeRobot/DDS integration, per-joint transport checks, and XR acceptance.
