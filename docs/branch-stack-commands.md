# Branch Acceptance Commands

These commands describe the keyboard-first stack, not the older installer pin.
They select frozen `archive/feature-stack-20260918/g1/*` snapshots, including
snapshots of the two active PR heads. Use `dev/g1-integration` for ongoing work
and `submit/keyboard-arm-control` for that live submission. Historical test counts
apply to the frozen commits, not future PR edits.
Use a configured environment with source imports verified. For the local workspace:

```bash
source ~/lerobot-dev/lerobot-dev.sh
cd ~/lerobot-dev/lerobot
```

Use the [reference migration](branch-reference-migration-20260918.md) for retired
branch names. Run `lerobot-check`
after each switch. Do not run multiple Hub/DDS simulators on the same domain.
Automated tests below are headless; visual commands are for an X-capable terminal
(such as `ssh -Y`). X/GLX support depends on the client/server setup.

## Exact Test Suites

Define these Bash arrays once. Paths are relative to the selected LeRobot checkout.

```bash
F=(tests/robots/test_unitree_g1.py tests/robots/test_unitree_g1_utils.py tests/teleoperators/test_unitree_g1_teleoperator.py)
K=(tests/utils/test_keyboard_input.py tests/teleoperators/test_unitree_g1_keyboard.py tests/integration/test_unitree_g1_keyboard_hub.py tests/integration/test_unitree_g1_keyboard_cli.py)
B=(tests/robots/test_unitree_g1_embodiments.py tests/robots/test_sonic_whole_body.py)
S=(tests/robots/test_unitree_g1_simulation.py tests/integration/test_unitree_g1_mujoco_runtime.py tests/integration/test_unitree_g1_keyboard_native.py)
C=(tests/robots/test_unitree_g1_cartesian_control.py tests/robots/test_unitree_g1_kinematics.py tests/robots/test_unitree_g1_action_processor.py)
X=(tests/teleoperators/test_unitree_g1_xr.py tests/integration/test_unitree_g1_xr_mujoco.py)
V=(tests/teleoperators/test_unitree_g1_xr_video.py tests/integration/test_unitree_g1_xr_video_mujoco.py)
H=(tests/robots/test_unitree_g1_hands.py)
R=(tests/robots/test_unitree_g1_brainco_hands.py)
```

After each switch, set the test flags (the workspace helper clears some flags):

```bash
export MUJOCO_GL=egl
export G1_KEYBOARD_HUB_TESTS=1 G1_RENDER_TESTS=1 G1_INTEGRATION_TESTS=1 G1_BRAINCO_SDK_TESTS=1
```

`G1_KINEMATICS_ASSETS` and `CYCLONEDDS_HOME` come from the workspace activation.
Otherwise set them to your pinned model directory and compatible native CycloneDDS
installation. Install the Unitree SDK, Hub simulator dependencies, and BrainCo SDK
before running their suites. Video GPU verification additionally needs Isaac Teleop
and offscreen Vulkan/CUDA. Required integration skips are not acceptance.

Verify the SDK's native CRC package data before Hub tests (no DDS participant or
hardware is opened by this command):

```bash
python -c 'from unitree_sdk2py.utils.crc import CRC; from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_; print(CRC().Crc(unitree_hg_msg_dds__LowCmd_()))'
```

Some SDK wheels omit `unitree_sdk2py/utils/lib`. If loading the CRC library fails,
restore that directory from the exact SDK source revision installed in the
environment, or use a correctly packaged/editable SDK installation. Our local
`lerobot-dev` copy was repaired from commit
`65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5`, without changing dependency versions.
Import-only SDK checks do not detect this problem.

| Selected branch | Exact branch-local pytest command |
|---|---|
| `archive/feature-stack-20260918/g1/bugfixes` | `python -m pytest -q "${F[@]}"` |
| `archive/feature-stack-20260918/g1/keyboard-arm-control` | `python -m pytest -q "${F[@]}" "${K[@]}"` |
| `archive/feature-stack-20260918/g1/embodiments` | `python -m pytest -q "${F[@]}" "${K[@]}" "${B[@]}"` |
| `archive/feature-stack-20260918/g1/simulation` | `python -m pytest -q "${F[@]}" "${K[@]}" "${B[@]}" "${S[@]}"` |
| `archive/feature-stack-20260918/g1/cartesian-control` | `python -m pytest -q "${F[@]}" "${K[@]}" "${B[@]}" "${S[@]}" "${C[@]}"` |
| `archive/feature-stack-20260918/g1/xr` | `python -m pytest -q "${F[@]}" "${K[@]}" "${B[@]}" "${S[@]}" "${C[@]}" "${X[@]}"` |
| `archive/feature-stack-20260918/g1/xr-video` | `python -m pytest -q "${F[@]}" "${K[@]}" "${B[@]}" "${S[@]}" "${C[@]}" "${X[@]}" "${V[@]}" -k 'not real_offscreen_delivery_and_recovery'` |
| `archive/feature-stack-20260918/g1/hand-support` | `python -m pytest -q "${F[@]}" "${K[@]}" "${B[@]}" "${S[@]}" "${H[@]}"` |
| `archive/feature-stack-20260918/g1/brainco-hands` | `python -m pytest -q "${F[@]}" "${K[@]}" "${B[@]}" "${S[@]}" "${H[@]}" "${R[@]}"` |

After sequential merge 7, run the video-stage command plus `"${H[@]}"`; after
merge 8 add both `"${H[@]}" "${R[@]}"`. Run the separate GPU gate at stages 6-8:

```bash
G1_VIDEO_GPU_TESTS=1 python -m pytest -q tests/teleoperators/test_unitree_g1_xr_video.py \
  -k real_offscreen_delivery_and_recovery
```

## Stage 1: Upstream G1-29 Keyboard

```bash
lerobot-switch archive/feature-stack-20260918/g1/keyboard-arm-control
MUJOCO_GL=glfw lerobot-teleoperate \
  --robot.type=unitree_g1 --robot.is_simulation=true \
  --robot.sim_publish_images=false --robot.sim_onscreen=true --robot.cameras='{}' \
  --teleop.type=unitree_g1_keyboard --display_data=true --display_mode=rerun
```

Do not add a locomotion controller. The existing Hub simulator's default torso
support remains enabled; this is not walking. No local URDF argument or IK is needed.
The Hub acceptance fixture pins `68459ed68f6f68e1f661091dfcb6ebce44681aec`;
the ordinary CLI follows the configured Hub repository's default revision.

Press Enter to enable, L/R to select an arm, 1-7 to select shoulder pitch/roll/yaw,
elbow, wrist roll/pitch/yaw, then +/- (or =) to jog. Space holds the measured pose
and disables jogging; Escape or Ctrl+C exits. Each input is a discrete 0.02 rad
increment, capped at 10 events/second with no backlog. Key release stops increments,
not holding torque. The last target remains active until hold/re-enable or exit.

Pass: all 14 joints respond with the expected sign, the other arm is unaffected,
startup/re-enable does not jump, limits are respected, and shutdown completes.
Joint limits are not collision avoidance; physical use through this CLI is rejected.

The same command works in a focused SSH terminal without X when `MUJOCO_GL=egl`,
`--robot.sim_onscreen=false` and `--display_data=false` are used. The PTY integration
test exercises this real CLI path, not a replacement launcher.

## Stage 2: Embodiments

```bash
lerobot-switch archive/feature-stack-20260918/g1/embodiments
MUJOCO_GL=glfw lerobot-teleoperate \
  --robot.type=unitree_g1 --robot.embodiment=g1_29 --robot.is_simulation=true \
  --robot.sim_publish_images=false --robot.sim_onscreen=true --robot.cameras='{}' \
  --teleop.type=unitree_g1_keyboard --display_data=true --display_mode=rerun
```

Pass: identical G1-29 behavior. G1-23 configuration/mapping is unit-tested here;
this branch alone still rejects its simulator connection. Do not interpret that
guard as a failed G1-23 dynamics test. Keyboard layout is derived from the robot
configuration; there is no second `--teleop.embodiment` selection.

## Stage 3: Native Simulation

```bash
lerobot-switch archive/feature-stack-20260918/g1/simulation
python examples/unitree_g1/prepare_cartesian_assets.py --output "$G1_KINEMATICS_ASSETS" --meshes
MODEL=g1_29
MUJOCO_GL=glfw lerobot-teleoperate \
  --robot.type=unitree_g1 --robot.embodiment="$MODEL" --robot.is_simulation=true \
  --robot.simulation_urdf="$G1_KINEMATICS_ASSETS/$MODEL.urdf" \
  --robot.simulation_mesh_dir="$G1_KINEMATICS_ASSETS/meshes" \
  --robot.gravity_compensation=true \
  --robot.sim_publish_images=false --robot.sim_onscreen=true --robot.cameras='{}' \
  --teleop.type=unitree_g1_keyboard --display_data=true --display_mode=rerun
```

Repeat with `MODEL=g1_23`. G1-23 has five joints per arm: keys 6/7 do not produce
commands. Pass: correct model, all available joints move independently, both arm
mappings/directions/limits agree, and holding/re-enabling/exiting work. Rerun shows
joint telemetry, not robot camera frames in this mode. This backend is fixed-base
and collision-free, and does not simulate locomotion or articulated hands.

## Stage 4: Cartesian

```bash
lerobot-switch archive/feature-stack-20260918/g1/cartesian-control
MUJOCO_GL=egl python examples/unitree_g1/verify_cartesian_control.py \
  --assets "$G1_KINEMATICS_ASSETS" --camera-azimuth -135
```

Pass: both panels show translation/orientation sweeps with bounded unreachable
targets. This is kinematic playback, not motor dynamics. Add `--headless` for
automated verification and inspect the generated report.

## Stages 5-6: XR And Video

At stage 5 use the X suite for headless controller-to-physics verification.
The integrated interactive XR example is introduced on the video branch, not the
XR-only branch. This corrects the earlier proposed stage-5 launcher command.
After starting CloudXR separately and configuring `XR_RUNTIME_JSON` as appropriate:

```bash
lerobot-switch archive/feature-stack-20260918/g1/xr-video
python examples/unitree_g1/run_xr_simulation.py \
  --assets "$G1_KINEMATICS_ASSETS" --embodiment g1_29 --steps 0
python examples/unitree_g1/run_xr_simulation.py \
  --assets "$G1_KINEMATICS_ASSETS" --embodiment g1_29 --video --steps 0
```

Run sequentially, and repeat both with `g1_23`. Pass: independent clutches, both
arm motions/rotations, tracking-loss/re-engagement behavior, then changing fresh
headset images and camera recovery. `--headless` selects synthetic replay, not
live headset input. Starting an external process does not export its environment
back to this shell. X/headset review remains manual.

## Stages 7-8: Hands

Run H/R and their ancestor suites from the table. These verify composition and
driver contracts, not articulated hand simulation or physical hands. Neither
branch needs Cartesian or XR at runtime. No physical verification command is
authorized or implied by this non-physical acceptance plan.
