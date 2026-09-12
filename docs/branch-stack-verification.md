# LeRobot Branch Stack Verification

The contribution checkout is `../lerobot-upstream`, with origin
`git@github.com:MoissanClub/lerobot.git`. The original `../lerobot` and this
experimental repository's runtime remain separate and are not replaced by the port.
Fork `main` stays at upstream base `b6ec0060779550c0a157ae34feb89e0cf86012a8`.

## Recorded Result: 2026-09-11

**Passed:** all seven branch-local suites and all seven sequential merge suites
in a fresh clone of origin. No merge conflicts. Final acceptance commit:
`d5e400bcefeccc93ba956ce876530e5283512df1` on `integration/g1-acceptance`.
The review checkout is `../lerobot-g1-review`.

This repository's runtime, configurations, existing tests, and shell launchers
were preserved. Documentation plus the new branch-stack runner and evidence were
committed at `4a47665`. The original `../lerobot` still has its pre-existing local
changes; a clean contribution checkout does not imply a clean original checkout.

| Merge milestone | Passing regression tests | Separate GPU pixel tests |
|---|---:|---:|
| 1 | 128 | Not applicable |
| 2 | 181 | Not applicable |
| 3 | 195 | Not applicable |
| 4 | 222 | Not applicable |
| 5 | 235 | 1 |
| 6 | 246 | 1 |
| 7 | 275 | 1 |

Each regression run had one optional SONIC module skip (ONNX unavailable).
All required model/render/integration and installed BrainCo SDK checks ran.
Ruff lint/format passed at every stage; the new camera channel also passed mypy.
Both embodiments passed the headless gravity-on/off simulation examples and
100-frame synthetic XR runs, each publishing 50 camera frames. Native kinematic
playback produced 63 changing frames out of 64 for each model; workspace-probe
residuals remain explicitly best-effort rather than 5 mm convergence claims.

[Recorded commands, branch SHAs, and per-merge results](verification/lerobot-branch-stack-20260911.json).
Full logs, JUnit XML and playback output are retained locally under
`artifacts/branch-stack-headless/`; rerunning the tool regenerates them.
The tested Python environment was reused; the source checkout was fresh.
No X/headset/physical acceptance is claimed for this merged fork.

## Feature Branches

| Milestone | Branch | Source parent | Branch-local regression |
|---|---|---|---|
| 1 | `g1/embodiments` | fork main | 128 passed, 1 optional SONIC skip |
| 2 | `g1/cartesian-control` | embodiments | 181 passed, 1 optional SONIC skip |
| 3 | `g1/simulation` | Cartesian | 195 passed, 1 optional SONIC skip |
| 4 | `g1/xr` | Cartesian | 206 passed, 1 optional SONIC skip; installed SDK pipeline check |
| 5 | `g1/xr-video` | XR | 217 passed, 1 optional SONIC skip; 1 separate offscreen GPU test |
| 6 | `g1/hand-support` | Cartesian | 192 passed, 1 optional SONIC skip |
| 7 | `g1/brainco-hands` | hand support | 221 passed, 1 optional SONIC skip, including installed SDK contract |

These counts cover the plan's targeted regressions, not the entire LeRobot test
tree. Branches 4/5 intentionally do not depend on simulation in source history.
Their actual MuJoCo integration suites run after merging them with milestone 3.

## Reproduce From Origin

Use the conda-forge Pinocchio/CasADi environment documented in the fork's
`examples/unitree_g1/README.md`, with pytest, Ruff 0.14.1, mypy 1.19.1, MuJoCo, Pillow, LeRobot dependencies,
and `bc-stark-sdk==2.0.2`. Install Isaac Teleop using its existing example guide.
Vulkan/CUDA device access is required for the offscreen GPU gate. No X server,
headset, physical robot, or hand is required. The new checkout must not exist.

From this experimental repository:

```bash
python tools/verify_lerobot_branch_stack.py \
  --checkout ../lerobot-review-fresh \
  --output artifacts/branch-review-fresh \
  --assets /tmp/g1-cartesian-assets \
  --python /home/dwei/lerobot-sim/.venvs/lerobot-upstream/bin/python \
  --sdk-site-packages /home/dwei/.venvs/isaacteleop/lib/python3.12/site-packages \
  --gpu
```

The two interpreter/SDK paths above are this workstation's existing environments;
replace them for another machine. Omit `--sdk-site-packages` when the SDK is
installed in the selected Python environment. Prepare pinned assets with the
fork's `examples/unitree_g1/prepare_cartesian_assets.py --output PATH --meshes`.
The environment installation recipe itself has not been tested from a blank OS.

The runner:

1. Clones the remote and records exact remote branch SHAs.
2. Checks out each feature branch, checks Ruff lint/format on changed Python files,
   and runs its dependency-specific suite.
3. Creates `integration/g1-acceptance` from the recorded upstream base.
4. Merges each branch with `--no-ff`, in milestone order, and reruns cumulative
   regressions after every merge, including combined XR/MuJoCo and camera tests.
5. Runs both simulation launchers with gravity compensation on/off, native
   side-by-side kinematic playback, and both combined XR/camera replay examples.
6. Writes `report.json`, JUnit XML, and command logs. It checks imported source
   paths and rejects missing required tests/dependencies or unexpected skips.

It does not push, create GitHub PRs, delete/reset existing checkouts, or alter main.
Leaving out `--gpu` produces `passed_without_gpu_gate`, not full acceptance.
Actual GitHub PR creation and review remain separate from these local merge checks.

## Manual Merge Workflow

For your own PR-by-PR verification, create a new acceptance branch from the same
base (do not overwrite an existing reviewed acceptance branch). Merge the seven
branches in the order shown above using merge commits. The exact suites and their
dependencies are in `SUITES` and `LOCAL` in the runner; recorded command logs also
give the full commands for every tested commit. Keep all model/render/integration
flags enabled when running the associated suites. Do not squash shared ancestry.

For example, from a new clone with no local changes:

```bash
git fetch origin
git switch -c integration/g1-pr-review b6ec0060779550c0a157ae34feb89e0cf86012a8
git push -u origin integration/g1-pr-review
```

Target your seven internal PRs at that new branch, not the already-merged
`integration/g1-acceptance`. Merge one at a time and test before continuing.
The runner automates local merge verification; it does not manage GitHub PRs.

## Manual X and Headset Review

On the merged fork, with the same environment and assets:

```bash
PYTHONPATH=src MUJOCO_GL=egl python examples/unitree_g1/verify_cartesian_control.py \
  --assets /tmp/g1-cartesian-assets --camera-azimuth -135
PYTHONPATH=src MUJOCO_GL=egl python examples/unitree_g1/run_simulation.py \
  --assets /tmp/g1-cartesian-assets --embodiment g1_23
```

Repeat the simulator with `g1_29`. For headset input and video, start the external
CloudXR runtime, then use `examples/unitree_g1/run_xr_simulation.py --assets PATH
--embodiment g1_23 --video --steps 0` with `PYTHONPATH=src MUJOCO_GL=egl`.
Repeat for G1-29. These manual tests have not been performed on the merged fork.
Use the environment prepared by your CloudXR installation, including its OpenXR
runtime selection (`XR_RUNTIME_JSON` where required). Starting an external process
alone does not export its environment into the terminal running these examples.

## Scope and Remaining Gates

- Native simulation fixes legs/waist/fingers and disables collisions. It uses
  configured PD plus optional measured-pose gravity feedforward, not calibrated
  full-body dynamics. The existing experimental DDS launchers are not ported.
- XR device reading is embodiment-independent. `G1XRControl` selects the shared
  model solver; malformed/stale samples disengage and re-engagement rebases.
  An SDK-specific rollback drains the session's entry ExitStack after partial
  startup failure; it has a regression test and needs review on SDK upgrades.
- Camera transport is generic (`lerobot.cameras.frame_channel`), avoiding the
  gRPC dependency in `lerobot.transport`. Input/video share one OpenXR session
  in an isolated worker. Offscreen tests do not prove CloudXR/headset behavior.
- Optional hands use a `G1WithHands` Robot wrapper rather than modifying the body
  driver. BrainCo has explicit six-slot normalized actions; anatomical slot
  labels remain unverified because vendor examples conflict. SDK 2.0.2 uses the
  module-level `modbus_close(client)` API.
- BrainCo identity/unit/rate/timeout tests use a fake SDK. The installed SDK check
  only inspects its interface. Physical access defaults off. No hardware, tactile
  calibration, torque-disable, or emergency-stop acceptance is claimed.
- Broader experimental workspace, recovery, endurance, and native DDS cleanup
  work remain in the non-physical backlog. Passing this port does not close them.
