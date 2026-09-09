# unitree_g1_lerobot

Unitree G1 support for LeRobot across G1-29 and G1-23, with separate robot, XR,
and simulation work tracks. Simulation and physical-robot acceptance are distinct.

## Three Tracks

| Track | Code Owner | Current Status | Next Milestone |
|---|---|---|---|
| 1. Add G1-23 to the LeRobot stack | `unitree_g1_lerobot/robots/` | Native model/IK reviewed; configurable G1 class, sparse joint mapping, motor defaults, and gravity adapter tested | Integrate selected G1-23 control with the live simulator and verify DDS feedback |
| 2. XR support for LeRobot | `unitree_g1_lerobot/xr/` | G1-29 right-controller-to-simulation motion user-confirmed | Bind the existing XR bridge to embodiment selection; verify G1-23 control and G1-29 regression |
| 3. Simulation for G1 | `unitree_g1_lerobot/simulation/` | Live G1-29 path; reviewed native geometry/IK and motor benchmarks for both variants | Supply the native G1-23 live simulator backend |

The tracks share interfaces and acceptance tests, not duplicate control implementations.
G1-23 live `connect()` is still explicitly blocked until its simulator backend is
integrated. Robot camera streaming to the headset and physical-robot validation remain
pending.

**Roadmap:** [three-track project plan](docs/project-plan.md).
**Resume:** [handoff](docs/rung4-handoff.md).
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

This constructs the robot interface; it does not claim a working G1-23 live connection.
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

Each launcher has confirmation-based startup diagnostics. Put on the headset after
the services are ready. Current headset support supplies controller input only; the
robot image is a workstation Tk/X view, not video streamed to the headset.

See the [operator guide](docs/operator-guide.md) for one-time setup, headset connection,
diagnostic behavior, embedded/external simulator modes, and troubleshooting.

## Track 3: Simulation for G1

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

Next: Track 3 supplies the native G1-23 backend; Tracks 1 and 3 verify the scripted
IK/DDS/actuator loop; Track 2 then verifies headset operation. Camera feedback follows
control acceptance, before physical G1-29 and G1-23 work. See the
[acceptance sequence](docs/project-plan.md#cross-track-acceptance).
