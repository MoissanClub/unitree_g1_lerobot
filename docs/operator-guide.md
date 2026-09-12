# G1 and CloudXR Operator Guide

For a coworker's new fork-based setup, use the
[installer and single VR launcher](coworker-installation.md). It installs its own
LeRobot dependency and supervises three native service processes. The detailed
DDS commands below remain a separate, preserved workflow.

Supporting Unitree G1 integration in LeRobot for MuJoCo simulation and real robot workflows across G1-29 and G1-23 variants.

This guide preserves detailed launch procedures. The [three-track project plan](project-plan.md)
and [README](../README.md) define current ownership and status. Commands below run
from the repository root, not from this docs directory.

**Repository scope:** these are the preserved experimental DDS/CloudXR launchers.
The separate contribution stack is complete and headless-verified; use the
[branch verification guide](branch-stack-verification.md) for commands in
`../lerobot-g1-review`. Do not mix its environment/setup with the patch procedure
below. X/headset acceptance of the new fork remains a separate manual review.

## Repository Layout

```text
unitree_g1_lerobot/
  robots/          # Embodiment specs, IK, shared arm control
  simulation/      # MuJoCo viewers/simulator and DDS adapters
  xr/              # XR bridge and CloudXR controller smoke test
  diagnostics/     # shared/, backends/, simulation/, xr/, physical/ verification tools
assets/g1/         # Vendored robot assets
configs/           # CloudXR runtime configuration
docs/              # Validation ladder and installation notes
tests/             # robots/, simulation/, xr/, diagnostics/, fixtures/
run_*.sh           # Stable root launch commands
```

Run launchers as before; they enter the repository root and invoke Python modules.
For direct Python commands, run `python -m unitree_g1_lerobot.<area>.<module>` from
the repository root with the appropriate environment. Running nested Python files by
filename is not supported. Existing launchers use the established environments.
For the new configurable G1-23 class, first apply the LeRobot embodiment patch as
described in [architecture](architecture.md#configurable-g1-runtime-structure).

See the [three-track project plan](project-plan.md) for remaining work,
and [architecture and upstream boundaries](architecture.md) for contribution scope.

At the 2026-09-10 end-of-day checkpoint, migration `bbc08f4` and all seven comparison
launchers are verified. The next task is broader motion acceptance using the existing
live verification launcher, followed by recovery and performance work. Start from the
[latest handoff](rung4-handoff.md) and [ordered backlog](future-non-physical-work.md);
launch/setup procedures below remain reference instructions, not unfinished bring-up tasks.

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

Gravity compensation is ON by default in the bridge (including its startup
diagnostic motions) and in simulator startup diagnostics, for both embodiments.
Use `--no-gravity-compensation` on the bridge to disable feedforward during XR control;
the simulator option controls only its own startup diagnostic. The simulator does not
add duplicate compensation to received DDS commands. Native G1-23 idle/stale-command
holding retains its exact-model compensation. `run_isaac_teleop.sh` has no gravity
option and performs no robot motion or diagnostic.

If the MuJoCo arms feel too slow or too subtle, tune only the bridge command gains
and controller translation scale first:

```bash
./run_xr_g1_mujoco.sh --external-g1-sim --external-cloudxr --wait-for-cloudxr --no-wait --arm-kp-scale 2.0 --arm-kd-scale 1.5 --xr-pos-scale 3.0
```

Raise `--arm-kp-scale` for faster response. If the arm overshoots or shakes, raise
`--arm-kd-scale` or reduce `--arm-kp-scale`. Raise `--xr-pos-scale` when the target
changes but the visual motion is too small.

To debug headset motion, add `--debug-xr` to Terminal C. While moving the controller
and holding the clutch, each hand's `raw` position and `target` should change, with
nonzero combined joint delta `q_d`. If `raw` stays constant, CloudXR/OpenXR is reporting
button state but not changing controller position.

Terminal B, start CloudXR. This is a robot-independent service: no startup motion,
confirmation, DDS request, or bridge prerequisite. It can also start before the simulator
or bridge, without changing robot state:

```bash
cd ~/lerobot-sim/unitree_g1_lerobot
./run_isaac_teleop.sh
```

For noninteractive runs, use `--headless` on all three launchers. Simulator and bridge
startup motion still runs, but there is no viewer or confirmation prompt. CloudXR is
always headless and accepts that flag for convenience. On the simulator and bridge, use
`--skip-startup-diagnostic` or `SKIP_G1_STARTUP_DIAGNOSTIC=1` only when intentionally
skipping the motion. CloudXR no longer accepts `--embodiment` or
`--skip-startup-diagnostic`; it does not need a LeRobot checkout or the G1 conda environment.
The standalone robot diagnostic remains available for an otherwise idle simulator.

Restart the simulator and bridge processes between sessions. Repeated DDS sessions
inside one Python interpreter exposed a native callback teardown crash; the headless
verification uses process isolation as containment. This has not been conclusively
assigned to LeRobot, the Unitree SDK, or CycloneDDS bindings. See
[Finding 10](vr-teleop-g1-23-ladder.md#finding-10-dds-session-teardown).

Then put on the headset. In the headset browser, open
`https://nvidia.github.io/IsaacTeleop/client`, use the `Quest3` profile, enter the
workstation IP printed by `run_isaac_teleop.sh`, enter XR, and connect.

This rung uses the headset for controller input only. The MuJoCo robot view is the Tk/X
window opened by `run_g1_mujoco_dds_sim.sh`; the headset client may stay on its CloudXR
control/status screen because `run_xr_g1_mujoco.sh` creates a headless OpenXR session and
does not submit MuJoCo camera frames to VR yet.

Default behavior is conservative: before XR attaches, both arms hold the raised ready pose.
After XR attaches, `--hand-side both` is the default: left controller drives the left
wrist and right controller drives the right wrist. Each clutch defaults to
max(squeeze, trigger) as an independent hold-to-enable input (threshold 0.5).
Release, invalid pose, or tracking loss freezes only that arm's joint command;
the other controller can continue moving its arm. Re-engagement latches a fresh
controller origin. Both controllers share one XR session and one IK/DDS sender.
Do not start two bridges. `--hand-side left` and `--hand-side right` retain single-arm modes.
If neither selected controller has a valid tracked pose, the existing reconnect policy
still applies: with `--wait-for-cloudxr`, after `--tracking-timeout-s` (default 20),
the bridge holds the ready pose while recreating its XR session. A single lost
controller does not restart the session while the other remains tracked.

For headset review, use the same embodiment on the simulator and bridge. Move each arm
separately, then both together, including wrist rotation. Release one clutch while
moving the other, then re-engage after repositioning the released controller. Repeat
with one controller temporarily untracked. Inspect both `left` and `right` log lines.
G1-23's five-joint arms cannot match arbitrary position and orientation simultaneously.
Finger actuation and robot-camera video are not added by this change.

### Rung 3 Runtime Model

`run_isaac_teleop.sh` starts CloudXR as a separate process. The rung 3 script attaches to
that existing CloudXR/OpenXR runtime with `--external-cloudxr`.

MuJoCo has two modes. Without `--external-g1-sim`, `unitree_g1_lerobot/xr/xr_to_g1_mujoco.py` creates
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

Motor-driven verification is now available separately from the existing geometry/IK
viewers:

```bash
./run_compare_g1_29_motor_configs_no_gravity_compensation.sh
./run_compare_g1_29_g1_23_motor_configs_no_gravity_compensation.sh
./run_compare_g1_29_motor_configs_with_gravity_compensation.sh
./run_compare_g1_29_g1_23_motor_configs_with_gravity_compensation.sh
./compare_motor_config.sh
```

These run the same supported-arm physics suite and produce side-by-side playback, plots,
and numerical reports. See [motor configuration comparison](motor-config-comparison.md)
for the source methodology, test conditions, and interpretation of oscillation/tracking.

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

### Planning and Acceptance

Native live G1-23 is now available with:

```bash
./run_g1_mujoco_dds_sim.sh --embodiment g1_23
```

See [G1-23 live simulator](g1-23-live-simulator.md) for the viewer-first diagnostic,
Enter confirmation, supported-arm scope, and tests. Select the same `--embodiment`
on the simulator, XR bridge, and CloudXR launcher. All three support `--headless`
for noninteractive startup; see [headless verification](../README.md#embodiment-selection-and-headless-verification).

The user has reviewed the geometry, IK, and motor-comparison artifacts. The
configuration-based G1 structure and native supported-arm DDS backend are tested;
systematic per-joint/Cartesian control and headset acceptance remain pending.

See the [three-track project plan](project-plan.md) for current ownership and next work:
Track 3 maintains the live simulator; Track 1 verifies robot/DDS control; Track 2 verifies
XR integration. Robot-camera feedback and physical-robot validation are separate later
acceptance gates. This guide documents operation, not a second independent roadmap.
