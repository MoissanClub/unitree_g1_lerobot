# Motor Configuration Comparison

Two pairs of launchers run actual MuJoCo motor-driven arm tests, save quantitative results, then
continuously replay the recorded physics trajectories in a side-by-side Tk/X viewer.
They do not publish DDS commands and need neither CloudXR nor a headset.

## Six-Panel Summary

Verification checkpoint: all four pair launchers completed 34 scenarios and opened
their Tk viewers; the summary completed all 204 trials. Each report has the expected
unique raw traces and 34 previews. Sixteen regression tests passed with real-viewer
checks enabled, including pause/resume, navigation, restart, test selection, and
robot-pixel motion in every panel. The extended hold pose is collision-free on both
models, and the 1 kg demonstration has no torque saturation. These checks do not
establish hardware speed/payload ratings or complete Rung 4 DDS/XR acceptance.

Reproduce the regression checks from the project root in the `lerobot-g1` environment:

```bash
python -m unittest discover -s tests -v
G1_TEST_VIEWER=1 python -m unittest discover -s tests -v
```

The second command requires a working X display and EGL; the first skips only the
real-viewer interaction test.

```bash
./compare_motor_config.sh
```

| | LeRobot G1-29 | Derived G1-29 | Derived G1-23 |
|---|---|---|---|
| Top row | Compensation OFF | Compensation OFF | Compensation OFF |
| Bottom row | Compensation ON | Compensation ON | Compensation ON |

The summary runs the same 34 scenarios for all six configurations, then replays
them on a shared timeline in one 1200-by-760-pixel image plus the toolbar. These are
independent supported-arm simulations, not interacting robots in one physics world.
Each panel identifies its profile and compensation mode. Compare vertically to isolate
gravity feedforward, and compare the first two columns to isolate G1-29 gains.
Reports include all six cases; raw trace filenames include compensation mode.

All launchers accept `--tests`, `--no-view`, `--gui-seconds`, `--output-dir`, and the
same camera options. Summary panel sizes can be changed with `--width` and `--height`.
For example:

```bash
./compare_motor_config.sh --tests pitch_20deg_step extended_hold_0kg extended_hold_1kg
```

## Run

From an `ssh -Y` terminal:

```bash
cd ~/lerobot-sim/unitree_g1_lerobot
./run_compare_g1_29_motor_configs_no_gravity_compensation.sh
./run_compare_g1_29_g1_23_motor_configs_no_gravity_compensation.sh
./run_compare_g1_29_motor_configs_with_gravity_compensation.sh
./run_compare_g1_29_g1_23_motor_configs_with_gravity_compensation.sh
```

Run one at a time for review. Within each pair, the first compares Unitree-derived G1-29 gains (left)
against the current installed LeRobot G1-29 gains (right), on identical physics models.
The second compares Unitree-derived G1-29 (left) and G1-23 (right) on their native models.
All body motor settings are recorded in the two profiles, but this benchmark tests arms
only: pelvis, legs, and waist are rigidly supported, so their holding gains are not ranked.

The 34 tests run before the viewer opens. Playback repeats the suite until the window
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

All four launchers share the same suite. In addition to the original ten cases:

- Pitch and roll steps of 5, 10, and 20 degrees (six tests).
- Pitch and roll sweeps at 0.5, 1, and 2 Hz with 10-degree amplitude (six tests).
- Pitch and roll 20-degree point-to-point moves over 0.8, 0.4, and 0.2 seconds
  (six tests). Quintic interpolation gives bounded acceleration and zero endpoint velocity.
- Pitch and roll rapid out-and-back reversals, 0.2 seconds per leg (two tests).
- Extended-arm holds with 0, 0.25, 0.5, and 1 kg per hand (four tests). Native joint
  targets use shoulder pitch -1.2 rad, elbow 1.0 rad, and mirrored shoulder yaw
  0.6 rad to separate the hands; this is a long forward reach, not a
  claimed maximum workspace pose. These tests reset at the hold target before warmup.

Step reports include worst-arm time to 90%, 10-90% rise time, target overshoot in
percent, and settling in a 0.02 rad band. Crossing thresholds reference the commanded
initial/final angles; uncompensated sag can prevent reaching them. Null means not reached,
not zero. CSV/JSON also include active-joint RMSE and post-motion settling/peak error
for point-to-point and reversal cases. These tests are not a maximum safe speed search.

Extended holds report final-second mean vertical hand sag (the larger of the two
hands) and peak gravity torque divided by motor limits. Compare identical profiles
between OFF and ON runs to isolate compensation. Payload is known to the compensated
model. These are demonstration loads, not hardware payload ratings; inspect saturation
and gravity torque headroom before interpreting sag. NPZ files record required gravity
torque even with compensation OFF; PD torque is raw torque minus gravity feedforward.

For a shorter gravity demonstration, append to any launcher:

```bash
--tests extended_hold_0kg extended_hold_0.5kg extended_hold_1kg
```

- 500 Hz MuJoCo physics and PD feedback; commanded joint targets update at 250 Hz.
- Gravity is enabled in all four launchers. The `no_gravity_compensation` pair uses
  `tau = clip(kp*(q_command-q) - kd*dq, +/-torque_limit)`.
- The `with_gravity_compensation` pair adds `g(q_measured)` before clipping the total
  torque. Gravity feedforward runs at 500 Hz, equally on both comparison sides.
  A separate MuJoCo data object evaluates bias forces at the measured pose with zero
  velocity, excluding Coriolis/centrifugal terms and avoiding changes to simulation state.
- Feedforward uses the exact plant model, including known payload mass. This is an
  ideal-model baseline, not a test of mass-estimation errors or live IK feedforward.
  No acceleration feedforward or friction compensation is applied.
- Viewer labels, output directory names, and reports identify the compensation mode;
  NPZ traces include the gravity feedforward torque separately from total torque.
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
