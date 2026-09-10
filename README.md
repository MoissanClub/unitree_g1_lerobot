# unitree_g1_lerobot

Unitree G1 support for LeRobot across G1-29 and G1-23, with separate robot, XR,
and simulation work tracks. Simulation and physical-robot acceptance are distinct.

## Three Tracks

| Track | Code Owner | Current Status | Next Milestone |
|---|---|---|---|
| 1. Add G1-23 to the LeRobot stack | `unitree_g1_lerobot/robots/` | Configurable G1 class and native backend connected; basic DDS command/feedback tested | Systematic per-joint and IK/DDS acceptance |
| 2. XR support for LeRobot | `unitree_g1_lerobot/xr/` | Both-arm bridge; headless checks pass on both variants; dual-arm headset review complete | Systematic lifecycle testing and headset camera display |
| 3. Simulation for G1 | `unitree_g1_lerobot/simulation/` | Live G1-29 and supported-arm G1-23 backends; reviewed model/IK/motor benchmarks | Broader DDS/actuator timing and scripted trajectory acceptance |

The tracks share interfaces and acceptance tests, not duplicate control implementations.
G1-23 now runs as a native supported-arm DDS simulator. Robot-camera streaming to the
headset is implemented and user-verified in the VR headset;
systematic controller lifecycle testing, and physical-robot validation remain pending.

**Roadmap:** [three-track project plan](docs/project-plan.md).
**Resume:** [handoff](docs/rung4-handoff.md).
**Future experiments:** [non-physical acceptance and reliability work](docs/future-non-physical-work.md).
**Live acceptance:** `./run_verify_live_control.sh` runs headless joint and IK/DDS checks
on both embodiments; see [coverage and reports](docs/live-control-acceptance.md).
**Design:** [architecture and upstream boundaries](docs/architecture.md).
The [rung ladder](docs/vr-teleop-g1-23-ladder.md) preserves earlier acceptance milestones;
Rung 4 spans all three tracks and is not complete.

## Track 1: G1-23 in LeRobot

Use one public G1 implementation with embodiment-specific data and IK. There are no
parallel G1-29/G1-23 robot subclasses. Existing G1-29 callers keep their defaults.

Apply the reproducible extension to the adjacent LeRobot checkout once:

```bash
./apply_lerobot_embodiment_patch.sh
```

Then select the embodiment through configuration:

```python
from unitree_g1_lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config

robot = UnitreeG1(UnitreeG1Config(embodiment="g1_23", is_simulation=True))
print(robot.action_features)
```

The selected G1-23 factory now supports an embedded headless live connection.
The local import registers G1-23 with LeRobot. `LEROBOT_ROOT` overrides the patch target.
See [configuration and backend boundaries](docs/architecture.md#configurable-g1-runtime-structure).

## Track 2: XR Support for LeRobot

XR owns controller acquisition, engagement/clutch handling, frame semantics, and
orchestration into the selected robot interface. Robot-specific IK and joint mappings
belong in `robots/`; MuJoCo rendering belongs in `simulation/`.

The working G1-29 session sequence is: simulator, bridge, CloudXR, then headset:

```bash
# Terminal A, from ssh -Y:
./run_g1_mujoco_dds_sim.sh

# Terminal C:
./run_xr_g1_mujoco.sh --external-g1-sim --external-cloudxr --wait-for-cloudxr --no-wait

# Terminal B:
./run_isaac_teleop.sh
```

The simulator and bridge have confirmation-based startup diagnostics. CloudXR starts
without robot motion or confirmation and can run independently. Put on the headset after
the services are ready. Add `--camera` to the simulator and `--video` to the bridge
for robot-camera presentation through the shared graphics/controller OpenXR session.
The CloudXR command is unchanged. The user confirmed video in the VR headset.

The bridge now defaults to `--hand-side both`: each controller moves its corresponding
arm while its trigger or squeeze is held above 0.5. Release or tracking loss freezes
only that arm's joint command. Use `--hand-side right` or `--hand-side left` for the
previous single-arm modes. One bridge owns both arms; do not run two bridges together.
Use matching `--embodiment g1_23` on the simulator and bridge for G1-23; CloudXR has no embodiment option.

See the [operator guide](docs/operator-guide.md) for one-time setup, headset connection,
diagnostic behavior, embedded/external simulator modes, and troubleshooting.

### Embodiment Selection and Headless Verification

The simulator and XR bridge accept `--embodiment g1_29|g1_23` (default: `g1_29`). For a
headless session, start these in separate terminals in the order shown:

```bash
# Terminal A:
./run_g1_mujoco_dds_sim.sh --embodiment g1_23 --headless

# Terminal C, after the simulator finishes its diagnostic:
./run_xr_g1_mujoco.sh --embodiment g1_23 --headless --external-g1-sim --external-cloudxr --wait-for-cloudxr

# Terminal B, after the bridge reports steady-state listening:
./run_isaac_teleop.sh --headless
```

Use `g1_29` in the simulator and bridge commands for that variant. Run only one live simulator at a
time. The bridge selects registered IK, joints, gains, and the embedded simulator
factory; external simulator identity is checked before sending commands. CloudXR is
robot-independent: no embodiment, gravity-compensation, or startup-diagnostic options.

`--headless` removes local viewers and confirmation prompts, but still runs startup
diagnostics and uses real XR input. It does not imply `--mock-xr`. Omit it for visual
confirmation; the simulator needs `ssh -Y` to open its Tk window. All three launchers
accept `--duration-s N` for bounded runs. The bridge's duration includes XR waiting
after startup diagnostics; CloudXR's duration begins after service readiness.

Reproduce automated checks for both variants from an idle simulation/CloudXR session:

```bash
conda run --no-capture-output -n lerobot-g1 python -m unitree_g1_lerobot.diagnostics.verify_xr_headless
```

The harness removes `DISPLAY`/`WAYLAND_DISPLAY`, runs actual launchers, exercises
both arms separately and simultaneously with deterministic controller translation/orientation, checks
command/feedback motion, and starts real CloudXR and headless OpenXR sessions. It
also rejects mismatched embodiments; right-arm checks use feedforward OFF and
left-arm checks use selected IK-model gravity feedforward ON. The bridge is started
before CloudXR to exercise its runtime-waiting path. The harness
prints a temporary directory containing logs and JSON evidence and stops its processes.
Real OpenXR uses one session with both controller streams. This is not physical
headset tracking, simultaneous two-controller headset acceptance, or video verification.

Bridge gravity feedforward is ON by default for both embodiments. Use
`--no-gravity-compensation` to disable selected IK-model feedforward. Simulator startup
diagnostics also default to ON and accept the same opt-out. While following DDS commands,
the simulator applies incoming torque without adding a second compensation term.
`run_isaac_teleop.sh` needs no gravity option and performs no robot diagnostic.
The G1-23 simulator
still compensates gravity during its own startup and stale-command hold.

## Track 3: Simulation for G1

Local robot-camera capture is available for both embodiments: add `--camera` to the
simulator and run `./view_g1_camera.sh` in a separate `ssh -Y` terminal. It also works
with a headless simulator. Add `--video` to the external-simulator XR bridge for headset
presentation as a mono virtual monitor. See
[camera commands, frame contract, and verification](docs/camera-streaming.md).

Start the native G1-23 simulator from `ssh -Y`, without a headset:

```bash
./run_g1_mujoco_dds_sim.sh --embodiment g1_23
```

The Tk view opens before both arms raise. Verify the motion and press Enter in the
terminal to proceed to steady-state DDS listening. Only the arms are dynamic; pelvis,
legs, and waist are supported. See [live simulator details](docs/g1-23-live-simulator.md).
Use the matching `--embodiment g1_23` on the XR bridge. The user has completed the
dual-arm headset review; exhaustive tracking-loss and reconnect coverage is not implied.

Preserve the separate geometry, IK, and motor-physics verification artifacts:

```bash
./run_compare_g1_29_g1_23_urdf_mesh.sh
./run_compare_g1_29_g1_23_ik.sh
./compare_motor_config.sh
```

The motor summary has three columns: LeRobot G1-29, derived G1-29, derived G1-23;
two rows: gravity compensation OFF above and ON below. All five motor launchers use
the same 34-case suite. They compute trials and reports before opening the Tk replay.

Geometry/IK playback is not live physics. Motor benchmarks simulate supported arms
but do not test the live DDS/XR loop. See [all four paired motor launchers and the
summary](docs/motor-config-comparison.md) for controls, metrics, payloads, and limits.

## Repository Layout

```text
unitree_g1_lerobot/
  robots/          # Track 1: embodiment, IK, motor data, shared G1 integration
  xr/              # Track 2: XR input, clutch, retargeting orchestration
  simulation/      # Track 3: MuJoCo models, viewers, runtime and DDS adapters
  diagnostics/     # Cross-track startup checks and verification suites
assets/g1/         # Robot assets and pinned source provenance
configs/           # Motor profiles and CloudXR settings
patches/           # Reproducible changes to the adjacent LeRobot checkout
docs/              # Project plan, operator guide, architecture, acceptance history
tests/             # Contract, physics, and optional real-viewer regression tests
*.sh               # Stable root launch/setup commands
```

Run Python modules from the repository root using
`python -m unitree_g1_lerobot.<area>.<module>`; root launchers set their working directory.
Use the existing `lerobot-g1` environment for robot/simulation work and the separate
Isaac Teleop environment for CloudXR. The G1-23 class extension requires the patch above;
cloning this integration repository alone does not modify LeRobot.

## Verification and Next Work

The structural refactor passed 25 project tests (including real viewers) and 92
existing LeRobot G1 robot/configuration/teleoperator tests. Geometry, IK, and motor
comparison visual review is complete. This is not hardware or live G1-23 acceptance.

The native G1-23 extension passed a 28-test project run with DDS and viewer checks
enabled. The G1-29 launcher/viewer smoke test passed, with startup motion skipped.
Code/model mapping checks found no mismatch for either embodiment, and all nine
runtime contract tests passed. These checks do not replace live motion acceptance.

The XR extension passed the headless three-launcher matrix for both embodiments,
including simultaneous mock arm motion, and the project suite with DDS enabled.
The GUI-only test is skipped in headless runs.
DDS integration cases run in fresh processes to contain a native callback teardown
crash. Responsibility among LeRobot, the Unitree SDK, and CycloneDDS bindings is
not yet isolated; this is not a confirmed CycloneDDS transport bug or an SDK fix.
See [Finding 10: evidence and next investigation](docs/vr-teleop-g1-23-ladder.md#finding-10-dds-session-teardown).

Dual-arm headset visual review is user-confirmed. Remaining Rung 4
acceptance includes:

1. Preserve the implemented embodiment selection and headless launcher regression checks above.
2. Verify every active arm joint and scripted IK -> DDS -> actuator trajectories,
   including feedback, control timing, and stale-command behavior.
3. Record systematic engagement/release, invalid tracking, loss, and reconnection
   coverage on both embodiments, including G1-23's five-joint orientation limitations.

Robot-camera streaming is separate **Rung 4V**, after control acceptance and before
physical G1-29 and G1-23 work. See the
[acceptance sequence](docs/project-plan.md#cross-track-acceptance).
