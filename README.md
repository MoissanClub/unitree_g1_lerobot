# unitree_g1_lerobot

Supporting Unitree G1 integration in LeRobot for MuJoCo simulation and real robot workflows across G1-29 and G1-23 variants.

## Repository Layout

```text
unitree_g1_lerobot/
  robots/          # Embodiment specs, IK, shared arm control
  simulation/      # MuJoCo viewers/simulator and DDS adapters
  xr/              # XR bridge and CloudXR controller smoke test
  diagnostics/     # Startup checks and rung verification
assets/g1/         # Vendored robot assets
configs/           # CloudXR runtime configuration
docs/              # Validation ladder and installation notes
tests/             # Package and launcher regression checks
run_*.sh           # Stable root launch commands
```

Run launchers as before; they enter the repository root and invoke Python modules.
For direct Python commands, run `python -m unitree_g1_lerobot.<area>.<module>` from
the repository root with the appropriate environment. Running nested Python files by
filename is not supported. No package installation or environment rebuild is required.

See the [validation ladder](docs/vr-teleop-g1-23-ladder.md) for remaining Rung 4 work,
and [architecture and upstream boundaries](docs/architecture.md) for contribution scope.

## Isaac Teleop CloudXR Scripts

Use these scripts for the Isaac Teleop / CloudXR part of the ladder.

### 1. One-Time Setup

Run this once after cloning the repo:

```bash
cd ~/lerobot-sim/unitree_g1_lerobot
./setup_isaac_teleop.sh
```

The setup script:

- creates a clean Isaac Teleop virtualenv at `/home/dwei/.venvs/isaacteleop`
- installs the local LeRobot checkout with the Isaac Teleop-compatible dependencies
- writes `configs/cloudxr_quest3.env`
- verifies the CloudXR and LeRobot Isaac Teleop CLIs
- ends by asking you to run the smoke test

Defaults can be overridden:

```bash
LEROBOT_ROOT=/path/to/lerobot \
VENV_DIR=/path/to/venv \
PYTHON_BIN=/usr/bin/python3.12 \
./setup_isaac_teleop.sh
```

### 2. Start CloudXR For Each VR Session

Run this whenever you want to connect the VR headset:

```bash
cd ~/lerobot-sim/unitree_g1_lerobot
./run_isaac_teleop.sh
```

On the headset browser:

```text
https://nvidia.github.io/IsaacTeleop/client
```

Set the headset-side profile to `Quest3`, then connect to the workstation IP printed by
`run_isaac_teleop.sh`. The local CloudXR runtime also uses `Quest3` through
`configs/cloudxr_quest3.env`; both sides need to match.

### 3. Smoke Test

With `run_isaac_teleop.sh` still running, open another terminal and run:

```bash
source /home/dwei/.venvs/isaacteleop/bin/activate
cd ~/lerobot-sim/unitree_g1_lerobot
python -m unitree_g1_lerobot.xr.xr_controller_cloudxr_smoke_test --external-cloudxr
```

Expected result: the script connects to the existing CloudXR runtime and prints live
controller pose/squeeze values from the headset.

## Rung 3: XR Controller To G1 MuJoCo

Rung 3 joins the CloudXR controller path with the G1 IK/MuJoCo path. Use the
`lerobot-g1` conda environment through the repo-local wrapper; it keeps conda-forge
Pinocchio/CasADi first and appends the existing Isaac Teleop venv only for XR imports.

Before a headset run, verify the G1 MuJoCo visual path from an `ssh -Y` terminal:

```bash
cd ~/lerobot-sim/unitree_g1_lerobot
./run_g1_mujoco_keyboard.sh
```

Validate rung 3 without headset or CloudXR:

```bash
cd ~/lerobot-sim/unitree_g1_lerobot
SKIP_G1_STARTUP_DIAGNOSTIC=1 ./run_xr_g1_mujoco.sh --dry-run-ik
SKIP_G1_STARTUP_DIAGNOSTIC=1 ./run_xr_g1_mujoco.sh --mock-xr --duration-s 5 --no-wait
```

For the ergonomic real headset run, start the lightweight pieces before putting on the
headset.

Terminal A, start G1 MuJoCo as a standalone DDS simulator. It first prints
`raising robot arm` and pauses after the motion so you can verify the viewer before
steady-state simulation starts:

```bash
cd ~/lerobot-sim/unitree_g1_lerobot
./run_g1_mujoco_dds_sim.sh
```

Terminal C, start the XR-to-G1 bridge before CloudXR exists. It first simulates a
CloudXR/controller input and pauses so you can verify both hands face up. Then it
attaches to DDS, sends the G1 to the raised-arm ready pose, and keeps holding that
pose while it retries XR attach:

```bash
cd ~/lerobot-sim/unitree_g1_lerobot
./run_xr_g1_mujoco.sh --external-g1-sim --external-cloudxr --wait-for-cloudxr --no-wait
```

This command runs until Ctrl+C. For bounded smoke tests, pass `--duration-s N`.

If the MuJoCo arms feel too slow or too subtle, tune only the bridge command gains
and controller translation scale first:

```bash
./run_xr_g1_mujoco.sh --external-g1-sim --external-cloudxr --wait-for-cloudxr --no-wait --arm-kp-scale 2.0 --arm-kd-scale 1.5 --xr-pos-scale 3.0
```

Raise `--arm-kp-scale` for faster response. If the arm overshoots or shakes, raise
`--arm-kd-scale` or reduce `--arm-kp-scale`. Raise `--xr-pos-scale` when the target
changes but the visual motion is too small.

To debug headset motion, add `--debug-xr` to Terminal C. While moving the controller
and holding the clutch, `raw_d`, `target_d`, and `q_d` should become nonzero. If
`raw_d` stays zero, CloudXR/OpenXR is reporting button state but not changing controller
position.

Terminal B, start CloudXR. When Terminal C is already running, this asks the XR
bridge to raise both arms briefly, lower both arms, and hold that diagnostic pose
until you confirm it. This avoids Terminal B and Terminal C publishing conflicting
DDS arm commands before CloudXR starts:

```bash
cd ~/lerobot-sim/unitree_g1_lerobot
./run_isaac_teleop.sh
```

For non-interactive smoke tests, prefix any launcher with
`SKIP_G1_STARTUP_DIAGNOSTIC=1` to skip these confirmation pauses. If Terminal B
cannot see the XR bridge within a few seconds, it falls back to the direct DDS
lower-arm diagnostic.

Then put on the headset. In the headset browser, open
`https://nvidia.github.io/IsaacTeleop/client`, use the `Quest3` profile, enter the
workstation IP printed by `run_isaac_teleop.sh`, enter XR, and connect.

This rung uses the headset for controller input only. The MuJoCo robot view is the Tk/X
window opened by `run_g1_mujoco_dds_sim.sh`; the headset client may stay on its CloudXR
control/status screen because `run_xr_g1_mujoco.sh` creates a headless OpenXR session and
does not submit MuJoCo camera frames to VR yet.

Default behavior is conservative: before XR attaches, both arms hold the raised ready pose.
After XR attaches, the right controller drives the right wrist only, the left wrist holds
ready, and the clutch defaults to max(squeeze, trigger) as a hold-to-enable input.

### Rung 3 Runtime Model

`run_isaac_teleop.sh` starts CloudXR as a separate process. The rung 3 script attaches to
that existing CloudXR/OpenXR runtime with `--external-cloudxr`.

MuJoCo has two modes. Without `--external-g1-sim`, `unitree_g1_lerobot/xr/rung3_xr_to_g1_mujoco.py` creates
`UnitreeG1(UnitreeG1Config(is_simulation=True))`, and `robot.connect()` launches the
LeRobot G1 MuJoCo simulation inside that Python process. With `--external-g1-sim`, the
bridge skips simulator creation and only publishes IK-generated G1 joint targets onto DDS;
`run_g1_mujoco_dds_sim.sh` owns the MuJoCo process.

Process split:

```text
A: cd ~/lerobot-sim/unitree_g1_lerobot && ./run_g1_mujoco_dds_sim.sh
C: cd ~/lerobot-sim/unitree_g1_lerobot && ./run_xr_g1_mujoco.sh --external-g1-sim --external-cloudxr --wait-for-cloudxr --no-wait
B: cd ~/lerobot-sim/unitree_g1_lerobot && ./run_isaac_teleop.sh
Headset: connect to CloudXR after A, C, and B are ready
```

## Rung 4: G1-23 Embodiment Bring-Up

There are two side-by-side verification scripts for this rung.

First, inspect the two native models in a stationary neutral pose:

```bash
cd ~/lerobot-sim/unitree_g1_lerobot
./run_compare_g1_29_g1_23_urdf_mesh.sh
```

This is the Step 3 URDF/mesh verifier: the left panel shows G1-29 and the right panel shows the native G1-23 URDF compiled by MuJoCo. Both cameras use the same distance and angle, adjusted for the models' root heights. The window stays open until closed or interrupted.

Then verify Step 4 with the native G1-23 MuJoCo model:

```bash
./run_compare_g1_29_g1_23_ik.sh
```

This opens one Tk/X window with two MuJoCo panels:

- left: LeRobot's existing G1-29 IK on the G1-29 MuJoCo scene
- right: local G1-23 IK driving a native G1-23 MuJoCo model compiled from the vendored URDF

The Step 4 script continuously repeats a 24-second cycle: up/down (Z), forward/back (X), left/right (Y), then hand orientation changes, six seconds per phase. Both models receive the same target offsets relative to their respective ready poses: +/-12 cm in Z and X, +/-10 cm in Y. These are diagnostic target ranges, not a measurement of the complete reachable workspace. G1-23 has five joints per arm, so it cannot independently match every position and orientation target. Actual hand travel is printed for each phase.

The active phase appears on both panels. Frames are precomputed at startup and then replayed continuously; this verifies kinematics, not live physics or DDS control. Close the window or press Ctrl+C to stop. A positive `--duration-s` limits playback for automated checks.

For headless smoke testing:

```bash
./run_compare_g1_29_g1_23_urdf_mesh.sh --duration-s 1 --no-view --save-final-frame /tmp/g1_compare_urdf_mesh.png
./run_compare_g1_29_g1_23_ik.sh --duration-s 24 --control-hz 10 --no-view --save-phase-frames /tmp/g1_compare_phases --save-final-frame /tmp/g1_compare_step4.png
```

The scripts include a blank-panel sanity check and print per-panel render statistics.

Both launchers accept camera settings. The default is the robot's left-front (robot-local
+X forward, +Y left), with matching angles and distance in both panels:

```bash
./run_compare_g1_29_g1_23_ik.sh --camera-azimuth -135 --camera-elevation -10 --camera-distance 2.2
./run_compare_g1_29_g1_23_urdf_mesh.sh --camera-azimuth -135
```

Azimuth 180 is front, -90 is the robot's left, and +135 is its right-front.
Negative elevation looks down from above. Change defaults in `unitree_g1_lerobot/simulation/g1_compare_ik_viewer.py`'s
`parse_args()`; restart the launcher to apply camera changes to the precomputed frames.

Current limitation: the native G1-23 panel is a visual/kinematic MuJoCo model compiled directly from URDF. It is not yet wrapped as a LeRobot Gym/DDS simulator.

### Rung 4 Remaining Work

As of 2026-09-09, the user has completed visual review of the native models and the
side-by-side IK sweeps. Keep both verification scripts as regression artifacts.
Rung 4 is still incomplete: assigning joint positions for rendering does not verify
motor-driven physics, DDS commands, or measured feedback.

The remaining acceptance work is:

1. Build the live native G1-23 MuJoCo/LeRobot simulator, with actuators, gravity,
   inertia, damping, and appropriate contacts. Verify that it holds and reaches arm
   poses through motor commands rather than direct joint-position assignment.
2. Verify the command/observation round trip through DDS: each of the ten active arm
   joints must have the correct name, index, direction, limits, and measured feedback.
   Account explicitly for unused slots in the transport's joint layout.
3. Send the existing scripted Cartesian/orientation sweeps through IK and DDS to the
   live simulator, without a headset. Compare targets, commanded joints, measured joints,
   and measured hand poses. Check tracking, timing, stability, and command-loss behavior.
   Distinguish G1-23 IK orientation compromises from actuator tracking errors.
4. Select G1-23 in the existing XR bridge and validate live headset control, startup
   diagnostics, engagement/release, and tracking loss/disconnect behavior. Recheck the
   G1-29 path for regressions and document the G1-23 five-joint orientation limitations.

A keyboard-controlled Cartesian stage is optional, not a prerequisite: it supplies
targets to the same IK and does not isolate an additional failure domain. The scripted
DDS verification is the next useful device-free check.

Implementation details, controller settings, and numerical acceptance thresholds remain
for the next planning session. This update records scope only; no live G1-23 simulator
or additional launcher is claimed to exist. Physical robot validation remains Rung 5.

Robot-camera video in the headset is not implemented or verified. Current Tk/X windows
are workstation views; successful XR controller input does not establish a video return
path. The ladder now tracks this separately as **Rung 4V**, after G1-23 control acceptance
and before hardware work. Camera capture, XR display integration, reconnect behavior,
and latency/frame-rate verification remain to be planned.

Hardware validation is split into **Rung 5a: physical G1-29 baseline**, when available,
then **Rung 5b: physical G1-23**. Both variants are documented by LeRobot; using G1-29
first reuses this project's tested IK/simulation embodiment while isolating hardware
integration. Neither physical workflow has been validated here.
