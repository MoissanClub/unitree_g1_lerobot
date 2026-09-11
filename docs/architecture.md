# Code Organization and Upstream Boundaries

The repository uses a named `unitree_g1_lerobot` Python package so local modules do not
occupy generic top-level import names such as `robots` or `simulation`. Root shell
launchers remain the operator interface. Python entry points use `python -m` from the
repository root. This is a checkout-based integration workspace, not a new distribution
that replaces or vendors LeRobot.

Package filenames, comments, and identifiers describe functionality, not planning stages.
The XR entry point is `xr.xr_to_g1_mujoco`; isolated IK and simulated execution checks
are `diagnostics.simulation.verify_g1_ik` and `diagnostics.simulation.verify_g1_ik_mujoco`. The comparison
viewer's repeating motion profile is `cartesian_orientation_sweep`. Root operator
launcher names are unchanged; rung labels remain only in planning documentation.

## Ownership

The [project plan](project-plan.md) organizes work in three tracks. Track 1 is G1-23
support in LeRobot (`robots/`), Track 2 is XR support for LeRobot (`xr/`), and Track 3
is G1 simulation (`simulation/`). The rung ladder supplies cross-track acceptance
gates, not separate module ownership. Robot camera capture belongs to Track 3;
headset display/streaming belongs to Track 2. Their frame contract is shared work.

- `robots/g1_embodiments.py`: G1-23 definitions and IK, plus adaptation to existing
  LeRobot G1-29 IK. Keep both variants together while behavior is shared.
- `robots/control.py`: shared kinematics, ready targets, and arm command helpers.
  These do not import the XR bridge, CloudXR, or simulator launch code.
- `simulation/`: MuJoCo rendering and runtime integration. `dds.py` owns the external
  simulation connection helper; `native_g1.py` supplies supported-arm G1-23 physics/DDS
  and `native_g1_viewer.py` supplies its viewer-first startup verification.
- `xr/`: controller input, clutch handling, CloudXR attachment, and XR-to-robot orchestration.
  `both_controllers.py` extends the existing LeRobot XR adapter with two output streams
  from one source/session, reusing its lifecycle and base-frame transform. The bridge
  maintains per-hand tracking, clutch, and targets, then solves bilateral IK once and
  publishes one combined command. Named arm masks are converted to Pinocchio order;
  inactive arms retain their last command despite IK filtering/regularization.
- `diagnostics/`: reusable cases/metrics/reports under `shared/`, simulation session
  management under `backends/`, and domain-specific tools under `simulation/`, `xr/`,
  and `physical/`. The live verification entry point remains shared and simulation-only.
  See [verification organization](verification-organization.md) for boundaries and discovery.
- `simulation/robot_camera.py`: opt-in rendering in a child process, fed bounded state
  snapshots after physics steps. `simulation/camera_frames.py` defines dependency-light
  latest-frame IPC; `diagnostics/simulation/view_robot_camera.py` consumes it without robot control.
  See the [camera frame contract](camera-streaming.md). `xr/camera_display.py` uploads
  frames to an SDK VizSession quad. `xr/video_controller.py` owns graphics and input
  in one worker process/shared OpenXR session, separate from the parent IK/DDS loop.
- `assets/g1/`: robot assets, outside the code package; asset paths resolve from the
  repository location rather than the terminal working directory.
- `configs/`: runtime configuration. `CLOUDXR_ENV_FILE` still overrides the default
  `configs/cloudxr_quest3.env` used by `setup_isaac_teleop.sh` and `run_isaac_teleop.sh`.
  The legacy `start_quest3_cloudxr.sh` retains its LeRobot example configuration default.
- `docs/`: installation history and the validation ladder.

## Future LeRobot Contribution

Keep upstream changes reviewable by responsibility. Robot embodiment/IK work is a
candidate for LeRobot's existing Unitree G1 integration; simulation changes belong with
the environment implementation, and XR examples belong with the existing teleoperation
integration. Exact target locations and APIs must be agreed with maintainers before a PR.

Local workstation paths, CloudXR provisioning, runtime monkey patches, and Tk verification
tools are integration scaffolding. They should not become dependencies of upstream robot
classes. Do not copy a second LeRobot or Isaac Teleop implementation into this package.
Dependency packaging, licensing/provenance review, and upstream API cleanup remain
separate contribution tasks; this structural refactor does not claim to complete them.

The refactor preserves algorithm bodies, control defaults, DDS topics, and launcher names.
It extracts shared helpers from the XR bridge without changing their behavior. Remaining
G1-23 physics/DDS work stays in the [Rung 4 plan](vr-teleop-g1-23-ladder.md).

## Configurable G1 Runtime Structure

The local integration registers G1-23 with one shared LeRobot `UnitreeG1` class;
there are no parallel G1-29/G1-23 robot subclasses or duplicated DDS/control loops.
Apply the versioned upstream-facing extension once to the adjacent checkout:

```bash
./apply_lerobot_embodiment_patch.sh
```

`LEROBOT_ROOT` overrides the target checkout. The helper checks before applying,
recognizes an already-applied patch, and refuses incompatible trees without reverting
or resetting anything. `patches/lerobot-g1-embodiments.patch` contains only the shared
runtime/configuration changes; upstream code does not import this project's modules.
The patch was generated against LeRobot commit
`3f2c29ef7e44b1ddccbcda3b6a63939e53639e9e` and verified against a fresh temporary
baseline, including an idempotent second application.

```python
from unitree_g1_lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config

robot = UnitreeG1(UnitreeG1Config(embodiment="g1_23", is_simulation=True))
print(robot.action_features)
```

This import registers the local embodiment and re-exports the actual LeRobot classes.
It must precede decoding a G1-23 config or calling LeRobot's robot factory. The normal
`make_robot_from_config()` factory and config serialization work after registration.
Existing direct LeRobot imports and configs default to G1-29 unchanged.

The immutable `G1RuntimeSpec` selects active and arm enums, DDS-indexed motor defaults,
home defaults, model resource, IK/gravity factory, and simulation factory. The shared
class uses it for features, state reads, commands, reset/shutdown, and torque scattering.
G1-23 has 23 real joints and 10 arm joints but retains a 29-slot transport layout;
unused slots have zero defaults and are excluded from action/observation features.
Gravity torques are mapped by arm enumeration, not by subtracting a start index.

Configuration validates variant names, array sizes, finite/nonnegative gains, control
timestep, unused slots, and controller compatibility. G1-29 retains LeRobot's existing
gains, model factory, zero home pose, and simulation/hardware transport selection.
G1-23 selects the source-derived gains, native MJCF runtime, and existing URDF-based parametric IK.
Its added `solve_tau()` adapter accepts G1 arm order and returns torque in that order.
The live controller retains its existing command-pose gravity policy; this is not the
benchmark's measured-pose compensation and this refactor does not silently change it.

**Integration boundary:** G1-23 now has a native supported-arm live simulator factory
and an embodiment-selectable standalone launcher. Basic DDS commands/feedback,
identity checks, stale holding, and embedded lifecycle have been tested. See
[live simulator](g1-23-live-simulator.md). G1-23 hardware and G1-29-specific whole-body
controllers remain blocked. Full per-joint, Cartesian/DDS, and XR acceptance is still pending.

The simulator and XR bridge select either embodiment; all three session launchers support
headless operation. CloudXR is robot-independent and has no startup diagnostic or
embodiment parameter, and does not require LeRobot or the G1 conda environment.
Mock left/right motion, external identity mismatch rejection, and
real CloudXR/OpenXR startup passed for both variants. Headset acceptance is separate.

**DDS lifetime boundary:** the combined tests reproduced a native publication-matched
callback crash across repeated sessions in one interpreter. LeRobot's disconnect path
does not explicitly close its DDS channels; the SDK writer cleanup relies on Python
object deletion. These are investigation leads, not proof that either layer alone or
CycloneDDS's transport engine caused the crash. Integration tests use fresh subprocesses
as containment, not a fix. A minimal SDK-only and then direct-binding reproducer is the
next step; see [Finding 10](vr-teleop-g1-23-ladder.md#finding-10-dds-session-teardown).

The updated LeRobot patch also guards a missing image-publisher process during G1-29
headless shutdown. That separate cleanup bug is fixed; the callback crash remains open.

Tests: `tests/robots/test_g1_runtime.py` exercises defaults, config round-trips/factory,
active features, sparse commands/torques, real G1-23 gravity ordering, and early backend
guards without commanding hardware. Existing LeRobot G1 tests cover backward compatibility.

Refactor verification: 25 project tests passed with `G1_TEST_VIEWER=1`; 92 existing
LeRobot G1 robot/configuration/teleoperator tests passed. The six-panel summary also
completed a shoulder-step and loaded-hold smoke run with its Tk viewer. The LeRobot
test run needs `pytest` and `pyserial` in `lerobot-g1` (installed during verification).
From the LeRobot checkout, reproduce the upstream regression subset with:

```bash
python -m pytest tests/robots/test_unitree_g1.py tests/robots/test_unitree_g1_utils.py tests/teleoperators/test_unitree_g1_teleoperator.py -q
```

## Verification

From the repository root, use the existing `lerobot-g1` environment:

```bash
python -m unittest discover -s tests -v
./run_compare_g1_29_g1_23_urdf_mesh.sh --duration-s 1 --no-view
./run_compare_g1_29_g1_23_ik.sh --duration-s 24 --control-hz 10 --no-view
SKIP_G1_STARTUP_DIAGNOSTIC=1 ./run_xr_g1_mujoco.sh --dry-run-ik
```

The skip flag is only for unattended smoke tests; ordinary launchers still run their
confirmation-based startup diagnostics. DDS tests must use an otherwise idle simulation
session. Real headset acceptance is separate from import and layout verification.
