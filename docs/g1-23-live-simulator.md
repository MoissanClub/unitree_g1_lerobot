# Native G1-23 Live Simulator

From an `ssh -Y` terminal in the repository root:

```bash
./run_g1_mujoco_dds_sim.sh --embodiment g1_23
```

The launcher uses the existing `lerobot-g1` environment. It opens a Tk/X viewer before
the startup motion, prints `raising robot arm`, and raises both arms through actual
DDS motor commands. Press Enter in the terminal after verifying the motion. It then
prints that it is entering steady-state listening. Close the window or press Ctrl+C
to stop. No headset or CloudXR is required.

Run only one live G1 simulator at a time. The standalone launchers and native backend
share a process lock. Both variants announce their embodiment on
`rt/lerobot/g1_sim_identity`; the external DDS helper verifies G1-23 identity and rejects
an announced mismatch before sending commands. Legacy G1-29 simulators without an
announcement remain compatible with G1-29 clients. This is a local simulation check,
not a hardware discovery protocol or protection against arbitrary DDS publishers.

## What Runs

- Native pinned G1-23 MJCF and meshes, using the same model construction as the
  reviewed motor benchmark. The G1-23 IK continues to use its native URDF.
- Ten dynamic arm joints. Pelvis, legs, and waist are rigidly supported; their reported
  positions are fixed at zero. This is not walking, balance, or full-body validation.
- Target 500 Hz physics, two substeps per 250 Hz DDS/state update. In standalone mode,
  physics has its own thread and does not advance only when the viewer refreshes.
  These are target rates, not a hard real-time guarantee over SSH.
- `rt/lowcmd` input and `rt/lowstate` feedback on DDS domain 0, loopback `lo` only.
- Incoming PD gains, velocity targets, and feedforward torque are used as supplied;
  target positions and total torque are clipped to the native motor limits.
  No extra gravity compensation is added to active DDS commands.
- Before commands arrive, and after 0.5 seconds without valid commands, the simulator
  holds a latched measured pose using derived PD gains and exact-model gravity torque.
  This is an explicit simulation fallback, not a physical-robot stop policy.
- CRC, finite arm fields, nonnegative gains, and unused-slot checks reject malformed
  commands and standard G1-29 packets that enable missing G1-23 joints.

The visible diagnostic publishes native G1-23 DDS commands with gravity feedforward.
It holds the diagnostic target while waiting for Enter. After it releases command
ownership, the simulator's stale-command hold maintains the achieved arm pose.

## Gravity Compensation

Physical gravity remains enabled at `[0, 0, -9.81]` m/s^2. Gravity compensation
means additional motor torque, not disabling gravity in MuJoCo.

| G1-23 live phase | Gravity feedforward |
| --- | --- |
| Startup arm raising and waiting for Enter | ON by default, exact MuJoCo model at measured pose; `--no-gravity-compensation` disables it |
| Idle or stale-command hold | ON, exact MuJoCo model at measured pose |
| Following valid DDS commands | Sender-controlled: uses incoming `tau`, without extra compensation |

For a LeRobot sender, explicitly set `UnitreeG1Config(gravity_compensation=True)`
to enable its IK-model feedforward; the configuration default is `False`.
The XR launcher sets this to `True` by default for both embodiments; pass
`--no-gravity-compensation` to that launcher to disable it during teleoperation.
The simulator launcher's compensation option controls its startup diagnostic, not
incoming DDS commands or the native stale-command safety hold.
That controller evaluates gravity at the commanded pose, unlike the simulator's
measured-pose fallback. Do not assume these are identical compensation methods.

The motor comparison launchers explicitly select compensation ON or OFF through
their filenames. `compare_motor_config.sh` shows OFF in the top row and ON in the
bottom row. Physical gravity is enabled in both rows.

## Verification Commands

```bash
# Bounded real viewer and diagnostic, no prompt:
./run_g1_mujoco_dds_sim.sh --embodiment g1_23 --no-diagnostic-confirm --duration-s 8

# DDS/physics only:
./run_g1_mujoco_dds_sim.sh --embodiment g1_23 --no-view --skip-startup-diagnostic --duration-s 5

# Existing G1-29 path, still the default:
./run_g1_mujoco_dds_sim.sh --embodiment g1_29

# Run from an otherwise idle simulation session:
G1_TEST_DDS=1 G1_TEST_VIEWER=1 python -m unittest discover -s tests -v
```

G1-23 accepts `--save-frame /tmp/g1_23.png` to save the latest viewer image. Its control
rate is fixed at `--hz 250`; `--view-fps` changes rendering only. With no `--duration-s`,
the process continues until stopped. `SKIP_G1_STARTUP_DIAGNOSTIC=1` is also supported.

## Embedded Use and Acceptance Boundary

The G1-23 simulator factory is registered with the shared G1 class. The following is
also available without a separate simulator process:

```python
from unitree_g1_lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
from unitree_g1_lerobot.simulation.dds import patch_unitree_dds_config

patch_unitree_dds_config()
robot = UnitreeG1(UnitreeG1Config(embodiment="g1_23", is_simulation=True))
try:
    robot.connect()
    observation = robot.get_observation()
finally:
    robot.disconnect()
```

Embedded mode is headless and steps physics through LeRobot's existing state thread.
It cannot run alongside the standalone native simulator on the same local session.
Hardware connection remains blocked.

Repeated DDS sessions in one Python process exposed a native crash in the
publication-matched callback under the combined test suite. Callback lifetime or
teardown order is suspected, but ownership among LeRobot, the Unitree SDK, and
CycloneDDS bindings remains unresolved. DDS integration tests use fresh subprocesses,
matching the launchers; this is containment, not a root-cause fix. See
[Finding 10](vr-teleop-g1-23-ladder.md#finding-10-dds-session-teardown) for the debugger
evidence, LeRobot's missing explicit channel cleanup, and the SDK-only reproducer plan.
Restart the simulator/bridge process between sessions instead of relying on repeated
connect/disconnect in a long-lived interpreter.

Verified: viewer-first startup, interactive confirmation, both-arm movement from a
separate DDS sender, joint feedback, identity mismatch rejection, command-loss holding,
and embedded connect/disconnect. Systematic per-joint acceptance, scripted Cartesian
IK/DDS sweeps, timing thresholds, and bilateral headset acceptance remain Rung 4 work.
The XR launcher now accepts `--embodiment g1_23`. Both variants pass headless
separate and simultaneous simulated-controller motion and real CloudXR/OpenXR startup checks.
Right-arm headset control is user-confirmed on both. The bridge defaults to `--hand-side both`;
simultaneous two-controller headset acceptance is still pending.
See [headless session commands](../README.md#embodiment-selection-and-headless-verification).

### G1-29 Regression Check

Also tested with the existing backend:

```bash
./run_g1_mujoco_dds_sim.sh --embodiment g1_29 --skip-startup-diagnostic --duration-s 3
```

The Tk viewer opened and the process exited successfully. Explicit renderer cleanup
fixed the EGL teardown traceback found during this check. This was a launcher/viewer
smoke test: G1-29 startup motion and headset control were not repeated in this pass.
The combined project test run passed 28 tests with DDS and viewer checks enabled.
