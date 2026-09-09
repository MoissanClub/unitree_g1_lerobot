# Code Organization and Upstream Boundaries

The repository uses a named `unitree_g1_lerobot` Python package so local modules do not
occupy generic top-level import names such as `robots` or `simulation`. Root shell
launchers remain the operator interface. Python entry points use `python -m` from the
repository root. This is a checkout-based integration workspace, not a new distribution
that replaces or vendors LeRobot.

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
  simulation connection helper; the existing simulator remains G1-29 only.
- `xr/`: controller input, clutch handling, CloudXR attachment, and XR-to-robot orchestration.
- `diagnostics/`: startup motions, bridge diagnostic requests, and rung smoke tests.
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
G1-23 selects the source-derived gains, native URDF, and existing parametric IK.
Its added `solve_tau()` adapter accepts G1 arm order and returns torque in that order.
The live controller retains its existing command-pose gravity policy; this is not the
benchmark's measured-pose compensation and this refactor does not silently change it.

**Integration boundary:** G1-23 can be instantiated and its configuration, features,
mapping, and IK/gravity contracts tested, but `connect()` deliberately raises before
opening DDS because the live G1-23 simulator factory is not integrated yet. G1-23
hardware and G1-29-specific whole-body controllers are also explicitly blocked.
This is the completed structural refactor, not completion of live Rung 4 acceptance.
Next, provide the native G1-23 simulator factory and verify the full DDS loop, then XR.

Tests: `tests/test_g1_runtime.py` exercises defaults, config round-trips/factory,
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
