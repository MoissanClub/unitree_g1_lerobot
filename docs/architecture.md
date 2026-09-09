# Code Organization and Upstream Boundaries

The repository uses a named `unitree_g1_lerobot` Python package so local modules do not
occupy generic top-level import names such as `robots` or `simulation`. Root shell
launchers remain the operator interface. Python entry points use `python -m` from the
repository root. This is a checkout-based integration workspace, not a new distribution
that replaces or vendors LeRobot.

## Ownership

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
