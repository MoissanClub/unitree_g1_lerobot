# Coworker VR Simulation Installation

This operator path uses the LeRobot fork's native arm simulator, not the legacy
DDS launchers. Clone `MoissanClub/unitree_g1_lerobot`, then run from its root:

```bash
./install_g1_vr_sim.sh
./run_g1_vr_sim.sh --embodiment g1_29
# Or:
./run_g1_vr_sim.sh --embodiment g1_23
```

## Host Prerequisites

- Linux x86_64, Python 3, Git, and Miniforge/conda on PATH. Use
  `--conda /path/to/conda` if needed; no activation is necessary afterward.
- An NVIDIA GPU with working drivers and Vulkan/EGL support for CloudXR. These
  host packages are not installed or replaced by this script.
- Internet access to GitHub, Hugging Face, conda-forge, PyPI, NVIDIA, and PyTorch.
  Allow substantial disk space for the isolated environment and GPU packages.
- For the desktop spectator window, a local X display or working `ssh -Y`.
- Headset network access to the host's CloudXR ports. Configure the firewall
  deliberately; this installer does not change firewall rules or request sudo.

The installer clones `MoissanClub/lerobot`'s `integration/g1-acceptance` branch
and checks out verified commit `d5e400bcefeccc93ba956ce876530e5283512df1` detached.
The correct spelling is **acceptance**, not `acceptanc`. An existing clean checkout
at that exact commit is reused; a different or modified checkout is never reset.

It creates `.vr-sim/env` with Python 3.12, conda-forge Pinocchio 3.9.0/CasADi 3.7.2,
LeRobot, MuJoCo 3.12.0, Tk/Pillow, ZeroMQ, Isaac Teleop 1.3.132rc1, and CUDA 12.8
PyTorch 2.11.0. It verifies real offscreen Vulkan/CUDA pixel delivery, downloads
pinned models/meshes, and runs synthetic camera/control verification for both
embodiments before writing `.vr-sim.json`. A dependency
snapshot is saved as `.vr-sim/pip-freeze.txt`. Transitive dependencies are resolved
at installation time; this is not a complete cross-platform lockfile.

Use `--prefix /absolute/path` to place dependencies elsewhere. The operator config
remains in the repository root. These generated paths are local and ignored by Git.
Rerunning installation reuses the environment and reruns checks; it never clears
an environment, changes the original adjacent LeRobot checkout, or applies the
experimental runtime patch. Do not run two installers concurrently.

## Three Services, One Launcher

`run_g1_vr_sim.sh` supervises three service processes implemented by
`tools/g1_vr_service.py`:

1. **simulator:** selected LeRobot/MuJoCo embodiment, fixed-base arm dynamics,
   gravity and gravity feedforward enabled, an isolated spectator Tk child, robot-camera
   publication, and local command/feedback endpoint.
2. **cloudxr:** the installed Isaac Teleop `CloudXRLauncher`, with the existing
   `configs/cloudxr_quest3.env` profile. It exports its resolved OpenXR environment
   for the bridge and refuses to take over an already-listening runtime.
3. **bridge:** shared LeRobot IK, independent left/right clutch control, and
   camera delivery through one shared input/video OpenXR session.

The simulator must become ready first; CloudXR becomes ready before the bridge
opens OpenXR. The launcher then tells the operator to connect the headset. These
are **new service entry points**, not calls to the old `run_g1_mujoco_dds_sim.sh`,
`run_xr_g1_mujoco.sh`, or `run_isaac_teleop.sh`. Those scripts remain unchanged for
the previously reviewed experimental DDS workflow.

Commands use a private per-run Unix IPC endpoint, not a network robot transport.
Stale/mismatched/out-of-limit commands are rejected. A command timeout holds the
measured arm position. This is simulation behavior, not a physical safety system.
Legs/waist/fingers are fixed and collisions are disabled in the native fork model.

The Tk child only reads latest-frame snapshots; X forwarding and window resize
cannot block the simulator's command loop. A crashed viewer does not stop headset
control. Closing the viewer normally requests shutdown of the entire session.
Expired valid commands are discarded with measured-position holding and a clutch
reset. On a transport timeout, the bridge replaces its request socket and queries
fresh feedback rather than resending the old target. Persistent loss of feedback
still fails the session after a bounded recovery interval (about 10 seconds).

Review the NVIDIA CloudXR EULA before first live startup. If you accept it, run:

```bash
./run_g1_vr_sim.sh --embodiment g1_29 --accept-cloudxr-eula
```

Subsequent runs use the SDK's saved acceptance. The script never accepts the EULA
implicitly. Set `CLOUDXR_ENV_FILE` to use a custom runtime profile.

Connect through `https://nvidia.github.io/IsaacTeleop/client` using the workstation
IP. If prompted, accept the workstation's TLS certificate at
`https://WORKSTATION-IP:48322/`. Squeeze each controller independently to clutch
that arm; release to disengage. The video is a mono head-following display of the
torso-mounted robot camera, not stereoscopic or head-steered robot vision.

Ctrl+C stops the owned services. Per-service logs remain in `outputs/vr-sim-*`.
Startup failures stop the other owned processes and print recent logs. Do not
start an additional CloudXR launcher alongside this one.

## Verification Modes

```bash
# GPU rendering and synthetic dual-arm input, no X, OpenXR, or headset:
./run_g1_vr_sim.sh --embodiment g1_29 --headless
./run_g1_vr_sim.sh --embodiment g1_23 --headless

# Real CloudXR/headset/video, but no desktop Tk window:
./run_g1_vr_sim.sh --embodiment g1_23 --headless --live-xr
```

Replay runs 100 frames by default; `--steps N` bounds a session. In replay, the
CloudXR service is explicitly a no-op placeholder; a replay pass is not live
CloudXR or headset acceptance. Live mode runs continuously by default.

The installer checks the fork's combined example. The operator's three-process
path needs its own replay and live checks; headset and X review remain manual.

## Development Verification: 2026-09-11

- The final installer completed end to end in a fresh private checkout and conda
  environment, including pinned CUDA 12.8 and the graphics gate. Host NVIDIA
  drivers and Miniforge were already installed; this was not a blank-OS test.
- The final clean environment also passed the operator process tests and real
  headless CloudXR startup checks; no old editable LeRobot/SDK environment was used.
- Both operator replays completed 100 frames with changing, nonblank camera
  images: G1-29 motion 0.2630 rad; G1-23 motion 0.1197 rad.
- Both real headless CloudXR/OpenXR input/video startups completed 20 frames,
  with 10-11 distinct camera frames each and no connected-controller motion.
  No headset or X display was used; this does not prove headset reception.
- The offscreen graphics gate passed actual GPU pixel readback.
- Twelve command/readiness unit cases and three real process tests passed,
  including both embodiment replays and cleanup after simulator termination.

Run those focused tests after installation:

```bash
G1_VR_OPERATOR_TESTS=1 .vr-sim/env/bin/python -m pytest -q \
  tests/diagnostics/test_vr_operator.py tests/simulation/test_vr_operator_processes.py
```

Use the installed interpreter from `.vr-sim.json` when using a custom prefix.
The known SDK/runtime `XR_ERROR_SESSION_NOT_STOPPING` warning still appeared
during live shutdown despite process exit code 0. Do not interpret successful
startup or cleanup as a fix for that lifecycle issue. Headset and X acceptance
of this new operator path is still required before shipping it as fully reviewed.

## SSH Window Resize Fix

The original operator implementation ran Tk updates inside the physics/command
loop. A stalled forwarded X window could therefore age camera frames and commands,
and the bridge treated the expired command as fatal. The viewer now runs in a
separate process, and expired commands take the hold/rebase recovery path above.

Headless regression after this fix: **21 tests passed**, including blocked viewer
children on both embodiments and simulator pauses of 0.8 and 3 seconds exercising
expired-command and socket-timeout recovery. No real X resize or headset test was
performed automatically; that visual review remains with the operator.

Immediate workaround for an older checkout (no desktop spectator window):

```bash
./run_g1_vr_sim.sh --embodiment g1_23 --headless --live-xr
```

After obtaining the updated operator code, no dependency reinstall is required.
