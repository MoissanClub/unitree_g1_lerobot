# G1 Branch Stack Verification

## Verified Checkpoint

All eight branch-local suites and eight sequential merges passed without merge
conflicts after the September 18 simulation-before-Cartesian reorder.
The combined branch is `integration/g1-acceptance-20260918`; its exact commit
is `published_integration.commit` in the recorded report below. The fresh trial's
`final_commit` has an identical source tree but independent merge history.

Upstream base: `5aa74557f84c54d4b458f8b9643c5aa2982acfed`.
The [branch plan](lerobot_g1_branching_refactor_plan.md) owns the dependency graph
and acceptance goals; this guide owns test definitions, evidence and commands.

| Order | Branch | Tested tip | Branch-local passes | Cumulative passes |
|---|---|---|---:|---:|
| 0 | `g1/bugfixes` | `e272f385` | 96 | 96 |
| 1 | `g1/embodiments` | `0fc02288` | 164 | 164 |
| 2 | `g1/simulation` | `5de18be1` | 187 | 187 |
| 3 | `g1/cartesian-control` | `db9cdda5` | 243 | 243 |
| 4 | `g1/xr` | `49df1ffd` | 270 | 270 |
| 5 | `g1/xr-video` | `a3dc34c7` | 283 | 283 |
| 6 | `g1/hand-support` | `1ccea9e4` | 202 | 298 |
| 7 | `g1/brainco-hands` | `a13c95fa` | 232 | 328 |

No tests were skipped. Eight additional simulation checks exercise the standard
CLI without IK dependencies, zero/unsupported remote input, and mocked native
viewer lifecycle on both embodiments. The video branch and
cumulative stages 5-7 each also passed one separate offscreen GPU pixel test.
Ruff lint/format, camera-channel mypy, installed SDK contract checks and both
embodiments' headless launchers passed. No X, headset or physical acceptance is
claimed for this revision. These are targeted suites, not the full LeRobot test tree.

[Machine-readable commands and results](verification/lerobot-branch-stack-20260918.json).
Full logs/JUnit XML remain locally in `outputs/branch-stack-20260918-final/`.
The fresh source clone came from the local contribution repository; its exact
tested branch tips are now published on origin. The Python environment was reused,
so this does not establish clean-OS installation. The [September 11 report](verification/lerobot-branch-stack-20260911.json)
is historical evidence for the older installer pin, not the current stack.
The [September 17 report](verification/lerobot-branch-stack-20260917-bugfix-update.json)
is preserved as evidence before the dependency reorder.

## Test Suites

Paths below are relative to a checked-out LeRobot fork. Suite identifiers match
the branch plan and the runner's `SUITES` / `LOCAL` definitions.

| Suite | Pytest paths |
|---|---|
| F: standalone fix | `tests/robots/test_unitree_g1.py`, `tests/robots/test_unitree_g1_utils.py`, `tests/teleoperators/test_unitree_g1_teleoperator.py` |
| B: embodiment baseline | F plus `tests/robots/test_unitree_g1_embodiments.py`, `tests/robots/test_sonic_whole_body.py` |
| C: Cartesian | `tests/robots/test_unitree_g1_cartesian_control.py`, `tests/robots/test_unitree_g1_kinematics.py`, `tests/robots/test_unitree_g1_action_processor.py` |
| S: simulation | `tests/robots/test_unitree_g1_simulation.py`, `tests/integration/test_unitree_g1_mujoco_runtime.py` |
| X: XR | `tests/teleoperators/test_unitree_g1_xr.py` |
| V: video | `tests/teleoperators/test_unitree_g1_xr_video.py` |
| H: hands | `tests/robots/test_unitree_g1_hands.py` (standard G1 composition is also tested in B) |
| R: BrainCo | `tests/robots/test_unitree_g1_brainco_hands.py` |

After cumulative merge 4, add `tests/integration/test_unitree_g1_xr_mujoco.py`.
After cumulative merge 5, also add `tests/integration/test_unitree_g1_xr_video_mujoco.py`.
Cumulative stages retain all earlier suites. XR/video branch-local runs now also
include their integration modules because simulation is in their source ancestry.
Simulation and generic/BrainCo hands do not run or require Cartesian suites.

The runner enables `G1_KINEMATICS_ASSETS`, `G1_RENDER_TESTS=1` and
`G1_BRAINCO_SDK_TESTS=1`; combined XR runs also use `G1_INTEGRATION_TESTS=1`.
It runs the ordinary video suite excluding `real_offscreen_delivery_and_recovery`,
then runs that test separately with `G1_VIDEO_GPU_TESTS=1` when `--gpu` is selected.
Missing required dependencies/assets or unexpected skips fail acceptance.

The motor-command regression uses real Unitree SDK messages to serialize at the
publisher boundary and decode the transmitted fields. Install `unitree_sdk2py`
and its CycloneDDS dependency in the test environment: the two parametrized cases
must pass, not skip. No DDS participant or physical robot is created. They cover
configured and overridden gains/torque, zero velocity, DDS slot indexing and an
unchanged neighboring motor command.

## Reproduce Headlessly

Use the conda-forge Pinocchio/CasADi setup in the fork's
`examples/unitree_g1/README.md`, plus pytest, Ruff 0.14.1, mypy 1.19.1, MuJoCo,
Pillow, LeRobot dependencies and `bc-stark-sdk==2.0.2`. Isaac Teleop is required
for the SDK pipeline and video checks. Offscreen Vulkan/CUDA requires GPU access,
but no X server, headset, physical robot or serial hand is needed.

Prepare pinned assets from a fork checkout using that environment:

```bash
PYTHONPATH=src python examples/unitree_g1/prepare_cartesian_assets.py \
  --output ../.cache/g1-cartesian-assets --meshes
```

Then, from this `unitree_g1_lerobot` repository:

```bash
python tools/verify_lerobot_branch_stack.py \
  --remote git@github.com:MoissanClub/lerobot.git \
  --checkout ../lerobot-review-fresh \
  --output outputs/branch-review-fresh \
  --assets ../.cache/g1-cartesian-assets \
  --python /home/dwei/lerobot-sim/.venvs/lerobot-upstream/bin/python \
  --sdk-site-packages /home/dwei/.venvs/isaacteleop/lib/python3.12/site-packages \
  --gpu --integration-branch integration/g1-pr-review-20260918
```

Use new checkout and output paths for every run. Interpreter/SDK paths above are
this workstation's setup; replace them on another host. Omit `--sdk-site-packages`
if Isaac Teleop is installed in the selected Python environment. Source imports
must resolve to the new checkout, not an older editable installation.

The runner clones origin, records its tips, checks every branch independently,
then creates a fresh cumulative branch from the pinned base and merges orders
0-7 with `--no-ff`. It reruns cumulative suites after each merge and finishes
with both embodiments' headless kinematic, gravity-on/off and XR/camera examples.

It never pushes, opens PRs, resets/deletes existing checkouts or contacts hardware.
Without `--gpu`, the result is `passed_without_gpu_gate`, not full acceptance.
The recorded report pins tested commits; a new run tests the branch tips fetched
at that time and records their actual SHAs.

## Manual PR Trial

For a GitHub PR-by-PR trial, use a fresh clone and a new acceptance target:

```bash
git clone git@github.com:MoissanClub/lerobot.git lerobot-pr-review
cd lerobot-pr-review
git switch -c integration/g1-pr-review-20260918 5aa74557f84c54d4b458f8b9643c5aa2982acfed
git push -u origin integration/g1-pr-review-20260918
```

Open internal PRs against that target in orders 0-7, beginning with
`g1/bugfixes`. Merge one at a time with merge commits, fetch/pull the target,
and run the cumulative suites above before continuing. Do not squash shared
dependency ancestry. Upstream-facing stacked PR bases instead follow the source
parents in the branch plan.

For example, after the first merge, from the fork with its environment active:

```bash
export PYTHONPATH="$PWD/src"
export MUJOCO_GL=egl
export G1_KINEMATICS_ASSETS=/home/dwei/lerobot-sim/.cache/g1-cartesian-assets
export G1_RENDER_TESTS=1
export G1_BRAINCO_SDK_TESTS=1
python -m pytest -q tests/robots/test_unitree_g1.py \
  tests/robots/test_unitree_g1_utils.py \
  tests/teleoperators/test_unitree_g1_teleoperator.py
```

For later merges expand B and append the cumulative suites; enable
`G1_INTEGRATION_TESTS=1` from merge 4 onward. Video additionally needs the separate
GPU gate described above. The recorded report contains exact successful commands
for every stage, including lint, typing and SDK checks.

The already-merged `integration/g1-acceptance-20260918` branch is for combined
review, not a fresh PR trial. Preserve the older `integration/g1-acceptance`
branch and the installer pin. Preserve the September 17 integration branch too.

## Manual X And Headset Review

These checks are for the user and remain pending on the new cumulative commit.
From its checkout, activate the simulation environment and prepare the assets:

```bash
export PYTHONPATH="$PWD/src"
export MUJOCO_GL=egl
export G1_KINEMATICS_ASSETS=/home/dwei/lerobot-sim/.cache/g1-cartesian-assets
python examples/unitree_g1/verify_cartesian_control.py \
  --assets "$G1_KINEMATICS_ASSETS" --camera-azimuth -135
python examples/unitree_g1/run_simulation.py \
  --assets "$G1_KINEMATICS_ASSETS" --embodiment g1_23
```

Use an `ssh -Y` terminal for Tk/X and repeat the simulator with `g1_29`.
For live controllers/video, start the external CloudXR runtime and configure
the example terminal's OpenXR environment (`XR_RUNTIME_JSON` where required):

```bash
python examples/unitree_g1/run_xr_simulation.py \
  --assets "$G1_KINEMATICS_ASSETS" --embodiment g1_23 --video --steps 0
```

Repeat for `g1_29`. Do not add `--headless` to a live video run: that flag
selects synthetic replay, not headset input. An external launcher does not export
environment variables back into this terminal.

Record the exact commit and review geometry, both arms, rotations, independent
clutches, release/re-engagement, tracking loss and camera freshness/recovery.
Prior headset reviews of the installed release do not transfer to the new revision.
For supported combinations and physical gates, see the branch plan.

## Simulation-Only CLI Review

On `g1/simulation`, no Cartesian code is present. Prepare assets with the existing
downloader, then run the standard command below in an X-capable terminal:

```bash
MUJOCO_GL=glfw lerobot-teleoperate \
  --robot.type=unitree_g1 --robot.embodiment=g1_23 --robot.is_simulation=true \
  --robot.simulation_urdf="$G1_KINEMATICS_ASSETS/g1_23.urdf" \
  --robot.simulation_mesh_dir="$G1_KINEMATICS_ASSETS/meshes" \
  --robot.sim_publish_images=false --robot.sim_onscreen=true \
  --robot.gravity_compensation=true --robot.cameras='{}' \
  --teleop.type=unitree_g1 --teleop.id=simulation_review --display_data=true
```

Repeat with the `g1_29` embodiment and URDF. This is a fixed-base posture/viewer
check, not a GR00T locomotion demo. Rerun shows joint telemetry, not simulated
camera frames in this mode. The headless equivalent of this entry point is tested
with `sim_onscreen=false`, `display_data=false`, and a bounded `teleop_time_s`.
See the [migration note](branch-stack-update-20260918.md#existing-checkouts) before
reusing local branches created before the reorder.
