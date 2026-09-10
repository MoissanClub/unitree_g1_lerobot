# Simulation and Sim-to-Real Work Plan

## Planning checkpoint: 2026-09-11

Continue development in two workstreams, with proposed branch names `simulation`
and `sim-to-real` starting from a shared checkpoint. This document records the plan;
it does not establish that those Git branches exist or that hardware acceptance has passed.
The workstreams span the existing robot, XR, and simulation code ownership tracks.

- **Simulation:** follow the [non-physical acceptance plan](future-non-physical-work.md)
  and [handoff](rung4-handoff.md). Next are systematic per-joint DDS and numerical
  IK-to-physics acceptance, followed by recovery, performance, and cleanup investigations.
  Camera streaming is implemented and basic headset video was user-verified.
- **Sim-to-real:** validate physical G1-29 first, then G1-23. For each embodiment,
  progress from simple slow joint motion to scripted IK motion to XR teleoperation,
  with the connection and acceptance gates below.

Share trajectory generation, embodiment definitions, metrics, and report formats across
the workstreams. Keep hardware connection and actuation explicit, and preserve simulation
regressions when changing shared control code. The broader simulation backlog can continue
independently; each physical stage needs evidence for its own prerequisites. Simulation
acceptance and physical acceptance remain separately recorded.

## Stage 0: Read-only preflight and control handover

Implemented tooling: [physical preflight and lifecycle audit](physical-preflight.md).
Run `run_g1_physical_preflight.sh` for passive DDS observation and
`verify_g1_physical_preflight.sh` for offline verification. Hardware observations and
the reviewed command/stop contract remain pending; feedback success does not enable motion.

Before the first command, confirm the physical embodiment, network interface, DDS joint
mapping, fresh measured state, and robot control mode. Establish which controller owns
the arms and which maintains the legs, waist, and balance or physical support. Preserve
a single command publisher for the selected control interface.

Audit `connect()`, startup diagnostics, `reset()`, and `disconnect()` for implicit motion.
Initialize targets from measured joint positions and define the transition into command
ownership. Record the physical support arrangement and operator stop procedure. Specify
the intended response to stale feedback, lost commands, operator interruption, and normal
exit, including control handover. Holding and releasing the arms are different outcomes;
choose the behavior for the actual control mode rather than assuming process exit is a stop.

**Gate:** produce a read-only preflight report and a reviewed command/stop contract before
enabling bounded motion. A slow trajectory alone does not establish safe initialization,
gains, feedforward, or control ownership.

## Stage 1: Simple, slow joint motion on G1-29

Use this stack to command one joint on one arm through a small, smoothly ramped displacement
relative to its measured starting position, independently of IK. Keep other joints at their
intended held positions without taking over joints owned by another controller. Repeat on
the opposite arm before expanding motion coverage.

Define position, velocity, acceleration, duration, tracking-error, and feedback-freshness
bounds before running. Verify the hardware gain and gravity-feedforward settings explicitly;
the simulation benchmark does not establish those settings. Log commanded and measured
positions for all commanded joints, control timing, and the actual stop/handover outcome.

**Gate:** correct joint identity and direction, bounded measured response, no unintended
motion, and verified stopping behavior. The first implementation deliverable is a read-only
preflight plus this bounded joint-motion diagnostic.

## Stage 2: Scripted physical IK verification

Build `unitree_g1_lerobot/diagnostics/verify_g1_ik_physical.py`, comparable in purpose to
[the MuJoCo diagnostic](../unitree_g1_lerobot/diagnostics/verify_g1_ik_mujoco.py), and run
it on the physical machine after Stage 1 passes. Reuse shared IK and trajectory helpers
where appropriate; keep hardware startup, command limits, and shutdown explicit.

The current MuJoCo script is a smoke test, not a physical acceptance contract. It starts
from zero joint/model poses, includes zero targets for non-arm joints in its action,
occasionally prints one shoulder's error, prints `PASS` without numerical assertions,
and calls `os._exit(0)` after disconnect. Reconsider each behavior for the physical tool.

The physical diagnostic must:

- Start targets and IK seeds from fresh measured state, with explicit joint ordering.
- Command only the intended ownership scope and enforce joint limits and rate bounds.
  Slow Cartesian motion can still produce fast joint motion near singularities.
- Begin with small single-arm trajectories, then test the other arm and bilateral motion.
- Record target poses, commanded joints, measured joints, and measured-state FK poses.
  Separate IK residuals from actuator tracking error and monitor every commanded joint.
- Use explicit numerical pass/fail thresholds and a defined response to violations,
  invalid IK output, stale feedback, or interruption. Record failures and cleanup outcomes.

Encoder-based FK establishes model-consistent tracking. Independent visual or external
measurement is needed to establish actual Cartesian accuracy. Choose thresholds before
declaring acceptance; do not derive hardware payload or speed ratings from these tests.

**Gate:** bounded trajectories satisfy the declared tracking and timing criteria, with
durable logs and reproducible configuration. Record any unavailable measurement separately.

## Stage 3: Physical XR teleoperation on G1-29

Integrate a deliberate physical backend into the XR path: the current
`xr/xr_to_g1_mujoco.py` constructs `UnitreeG1Config(is_simulation=True)` and uses
simulation-specific startup/connection code. Preserve the existing simulation launchers
and reuse the shared robot and XR logic rather than duplicating the control implementation.

Before unrestricted teleoperation, exercise bounded failure/recovery cases: command
interruption, stale feedback, clutch release, invalid/lost tracking, and disconnect/reconnect.
Confirm the intended physical hold/handover behavior and absence of stale input replay.
Reconnection requires deliberate re-engagement with fresh reference poses.

Progress through one arm, the other arm, and bilateral control with conservative limits.
Retain one robot-command publisher and independent clutches. Record observed behavior,
timing, tracking errors, and stop/recovery outcomes. Camera or CloudXR failures must follow
the defined control contract; service readiness alone is not acceptance evidence.

**Gate:** the scripted-motion criteria remain satisfied during XR operation, and the
declared lifecycle cases pass before expanding the operating envelope.

## Repeat Stages 0–3 on G1-23

After the G1-29 sequence, repeat preflight, simple motion, physical IK verification, and
teleoperation on G1-23. Validate its hardware backend before enabling motion: the current
runtime explicitly blocks G1-23 hardware through its `hardware_supported` capability.
Removing that guard alone does not implement or validate hardware support.

Recheck sparse DDS slots, active joints, directions, control modes, gains, feedforward,
and stop behavior on the actual embodiment. Set separate position/orientation criteria for
its five-joint arms, which cannot independently satisfy all six Cartesian pose objectives.
G1-29 success provides a baseline, not inherited G1-23 acceptance.

## Evidence and open decisions

Archive software revisions (including the applied LeRobot patch), embodiment and interface,
physical setup, control ownership, gains/feedforward, trajectory settings, thresholds,
command/feedback traces, timing, operator observations, and stop/recovery results for each run.
Use durable verification artifacts rather than only temporary directories.

Exact motion bounds, thresholds, hardware support/control mode, and stop implementation
remain to be selected during preflight. Keep the DDS native teardown finding open;
fresh-process isolation contains it but is not a fix or a physical stopping mechanism.
Upstream contribution preparation remains separate from experimental acceptance.
