# Physical Deployment Gates

This guide owns staged hardware acceptance, not project branch strategy.
See the [execution plan](project-plan.md) for priorities and
[architecture](architecture.md#physical-control-and-sonic) for control boundaries.
Physical G1-29 is the preferred baseline when available, followed by separate G1-23
acceptance. Unavailable hardware must be recorded, not treated as a passed gate.

## Current Focus: G1-29 Arm-Only Software Plan

**Direction update, 2026-09-29:** follow Unitree's motion-mode architecture with
upstream G1-29 IK/static feedforward, GR00T in simulation and high-level stock
locomotion on hardware. New base: `work/g1-29-vr-teleop` from `g1/bugfixes` at
`321180e74`, checkout `/home/dwei/lerobot-sim/g1-vr-teleop`; see its
`docs/source/g1_vr_execution_plan.mdx`. No full-body dynamic compensation is
required before initial combined trials, but ownership and operating limits still
need validation. The candidate and commands below are preserved at `39e02d7d2`
on `work/g1-29-arm-hardware` as historical evidence. The new branch now contains
selectively migrated camera/arm backends and a Unitree-mapped runtime using upstream
IK. Its [manual test plan](https://github.com/MoissanClub/lerobot/blob/work/g1-29-vr-teleop/docs/source/g1_vr_manual_test_plan.mdx)
is authoritative for new runs. The old counts and commands below apply only to
the preserved checkpoint, not the new implementation.
The physical acceptance order remains simulation -> feedback -> camera -> shadow
-> hold -> bounded joints -> stationary VR arms -> locomotion/combined VR.

Updated 2026-09-29. Deliver independent left/right VR arm control in simulation and
on G1-29 hardware. No BrainCo actuation, walking commands, SONIC, or G1-23 work is
required for this milestone. Any physically attached hand still affects payload,
tool transforms and clearance; disabling its driver does not remove its mass.

The development integration reference checked for this plan is
`MoissanClub/lerobot:dev/g1-integration` at
`d89a1d0b5f628045cc73293bcdfa2dc6c0808549`. The BrainCo simulation branch is separate
and is not a prerequisite. Do not change a checkout used by a running process.
Use a short-lived work branch from the audited integration baseline, retain the
pinned simulation release, and extract small upstream submissions separately.

This section owns software implementation order. Stages 0-3 below own hardware
acceptance. The implementation checkpoint below distinguishes delivered software
from pending physical evidence. No document or software test authorizes motion.

### Implementation Checkpoint: 2026-09-29

Candidate checkout: `/home/dwei/lerobot-sim/g1-arm-hardware`, branch
`work/g1-29-arm-hardware`. The pinned integration and simulation release were not
modified. Detailed runnable commands and limitations live in the candidate's
`docs/source/g1_arm_sdk_validation.mdx`.

### Operator Test Order

The following table documents the **previous candidate**. The new runtime implements
all software stages, including physical VR arms and high-level velocity dispatch,
but none of its physical stages or headset/X acceptance has been executed here.
The new mapping uses terminal `r` + Enter to start, not squeeze-to-clutch. Its
simulator uses locked Dex3 inertias matching upstream IK, without finger actuation;
actual physical payloads must be reviewed. The supported arm diagnostic fixture
does not emulate stock balancing or firmware handover. Free-base GR00T is tested
separately. Do not run the old whole-body server with either physical backend.

Use the candidate's `docs/source/g1_arm_test_plan.mdx` for exact commands and
expected observations. The agreed order, retaining the discussion's step numbers,
is **1 -> 2 -> 3 -> 6 -> 4 -> 5 -> 7**:

| Phase | Tests | Current implementation |
| --- | --- | --- |
| Simulation | 1: VR controls MuJoCo | Existing XR simulation; manual headset baseline is separate from automated replay |
| Read-only physical | 2: feedback; 3: camera-only headset display; 6: XR shadow targets | Arm SDK read-only diagnostic plus new `validate_xr_readonly.py` camera/display/shadow commands; no motor publication |
| Bounded physical | 4: supervised hold; 5: single-joint excursion | Existing `validate_arm_sdk.py`, gated by reviewed contract and explicit motion opt-in; simulation-tested, not hardware-accepted |
| Physical XR | 7: supervised headset control | Not implemented in the hardware candidate; gated on previous results |

Camera capture uses the existing OpenCV camera API and same-host RGB channel;
display reuses the existing headset monitor. Optional explicit CloudXR startup
does not start a robot server. Shadow forces read-only feedback and logs IK/clutch
results without an activation/send path. Neither camera delivery nor shadow
requires `run_g1_server.py`. A fake-camera plumbing test and real MuJoCo shadow
test are available; actual PC2 camera/headset review remains required. The local
offscreen GPU attempt failed with CUDA out-of-memory while existing vLLM workers
occupied both GPUs; those unrelated processes were left untouched.

Hold and single-joint tests are **real torque-enabled hardware tests**, not
read-only or inherently low-risk operations. No physical runs were performed.

Read-only extension verification (2026-09-29): broad regression 268 passed,
2 skipped; final focused camera/shadow suite 7 passed. The actual headless XR
example completed 100 control frames and 50 camera frames with 0.2631 rad measured
motion. Broad report: `/home/dwei/lerobot-sim/g1-arm-readonly-regression.xml`.
The separate opt-in GPU run failed with CUDA out-of-memory and remains outstanding;
it is not included among passing tests.

| Software gate | Current evidence / boundary |
| --- | --- |
| Baseline | 118 existing focused tests passed before edits |
| Authority candidate | Official SDK arm7 example audited at `814556d15970dd2ecf1c9984e845ca02ab07e206`; `rt/arm_sdk`, slots 15-28 plus protocol weight 29, no MotionSwitcher or `rt/lowcmd`; actual firmware/waist behavior remains operator review |
| Shared measured state | Existing XR simulation example now reads arm positions from Robot observations, not MuJoCo state arrays; physical XR launcher remains a later gate |
| Read-only backend | Explicit `UnitreeG1Config.arm_sdk` path, subscriber-only connect, no automatic reset/activation, fresh state/mode/tick checks |
| Bounded execution | Explicit activation, measured-pose blend-in, complete arm-only commands, joint/displacement/rate/estimated-torque bounds and optional measured-pose gravity; local robot-side thread watchdog with latched faults |
| Diagnostic | `examples/unitree_g1/validate_arm_sdk.py`: offline template/dry-run, read-only, hold and single-joint tests; immutable report filenames and explicit hardware opt-in |
| Physics evidence | Same backend and message builder exercised through fake transport and real MuJoCo, including both elbows; not firmware emulation |
| Broad regression | Final run: 251 passed, one optional BrainCo installed-SDK audit skipped; includes existing model/IK/dual-arm XR regression and startup/shutdown fault tests. Local report: `/home/dwei/lerobot-sim/g1-arm-regression-final.xml` |
| Hardware | No physical connection, ownership acquisition or movement performed; begin with supervised read-only validation |

Normal exit/fault requests release using weight-zero packets; physical stop is not
asserted. The thread cannot enforce a watchdog after process/host death or lost DDS.
The operator must review firmware behavior and independent stop/support arrangements.
The official example also commands waist; this arm-only candidate requires explicit
confirmation that uncommanded waist fields preserve its stock controller. Do not
merely turn contract flags on to bypass these decisions.

The initial gravity-loading timing failure was caught by the simulation diagnostic
and fixed by loading the model before feedback collection/activation. Publisher
discovery now waits with weight zero before measured-pose activation. See the
candidate guide for other limits, report semantics and exact reproduction commands.

### 1. Freeze and Reproduce the Arm Baseline

- Record source SHA, Python/dependency versions, model/mesh pins, IK frames and
  current gains/feedforward settings. Re-audit current upstream before extracting PRs.
- Reproduce G1-29 joint, Cartesian, bilateral clutch and tracking-loss tests on the
  existing simulator. Preserve camera/headset operation as a separate regression.
- Inventory existing physical read-only tools and their source-specific evidence;
  do not carry their old audit hashes forward as current acceptance.

Deliverable: a reproducible manifest and passing baseline report. No hardware
connection is needed. Reuse existing tests rather than rebuilding the simulator.

### 2. Select and Specify Physical Arm Authority

- Audit the manufacturer's SDK/examples against the actual robot variant and
  firmware. Investigate the supported arm SDK path while stock control owns the
  lower body; verify its topic, message layout, ownership/blending fields and modes.
- Explicitly assign arms, waist, legs and balance to one owner each. Confirm whether
  arm-only operation is supported in the selected mode and physical arrangement.
- Specify acquisition, measured-pose initialization, release, timeout and operator
  stop behavior. Do not infer a safe handover from stopping publication.
- The current `run_g1_server.py` releases an active mode and publishes `rt/lowcmd`.
  Do not reuse that startup unchanged or label it an arm-only transport.

Deliverable: reviewed protocol/ownership contract and simulator/fake-transport
fixtures. This is a real decision gate: if the proposed arm-only mode is unavailable,
stop and select a supported arrangement rather than silently taking whole-body control.

### 3. Make Cartesian/XR Control Independent of the Simulator

- Keep one `UnitreeG1` configuration/interface and reuse existing G1-29 IK and XR
  processing. Separate simulator execution from physical transport internally.
- Replace direct `robot._native`/MuJoCo-state reads in the control orchestration
  with fresh measured arm observations in explicit joint order and units.
- Keep physics stepping/rendering in the simulation backend. Neither hardware
  execution nor the VR device reader should depend on a MuJoCo model instance.
- Define target-pose frames, clutch anchors, joint limits and rate limits once;
  preserve independent clutches and restart references from current measurements.
- Prefer LeRobot processor/robot conventions; prove standard-loop observation and
  action wiring before claiming a generic teleoperation CLI works end to end.

Deliverable: the same scripted Cartesian and synthetic XR cases execute through
the shared pipeline using simulation or fake hardware. No physical actuation.

### 4. Implement a Read-Only Physical Backend First

- Add explicit transport/interface/mode configuration; do not auto-select a robot.
- Connect, validate joint/state schema and expose timestamped measured feedback.
  Reject missing, stale or invalid state. Read-only mode creates no command writer,
  changes no robot mode and cannot call reset or send actions.
- Audit startup, exception cleanup and disconnect for implicit commands. Adapt
  the existing passive tools to the selected revision and record new source hashes.

Deliverable: offline lifecycle tests plus a bounded read-only hardware diagnostic.
This is the first hardware test: it verifies feedback only, not permission to move.

### 5. Add Bounded Arm Command Execution and Robot-Side Watchdog

- Make motion a deliberate opt-in after fresh state and explicit ownership checks.
  Initialize arm targets from measured positions, not zeros or a preset ready pose.
- Build commands according to the selected arm protocol. Assert that no non-arm
  joint targets or unauthorized mode changes can be emitted; retain any required
  protocol ownership fields explicitly rather than treating them as ordinary joints.
- Validate finite targets, model bounds, per-joint velocity/acceleration limits,
  gains, torque/feedforward bounds, and measured tracking error on every update.
- Place the watchdog at the robot-side command owner so it also covers laptop/XR
  process death and network loss. Reject old/out-of-order commands and stale feedback.
- Implement the reviewed hold/handover response. Do not assume zero torque, damping,
  holding position or process exit is universally safe. Re-enable only deliberately.
- Use measured-pose gravity compensation where validated for this authority mode;
  avoid duplicating compensation already supplied by the robot controller.

Deliverable: a command backend with deterministic fault-injection tests. It must
not import the BrainCo SDK or require the SONIC/VLA stack.

### 6. Build the First Small-Motion Diagnostic

- Implement a bounded joint diagnostic in the physical diagnostics area, separate
  from the headset launcher. Offer dry-run and read-only modes before motion mode.
- Require a named arm/joint, measured-relative displacement, duration and an explicit
  reviewed limits/configuration file. Motion mode requires explicit operator enablement.
- Ramp one joint, monitor all commanded joints, then use the agreed return/hold/
  handover sequence. Never automatically move the opposite arm or run a ready pose.
- Write durable reports: source/model/config hashes, joint identity, measured start,
  command/feedback traces, timing, violations and actual cleanup outcome. Failure,
  cancellation and incomplete execution must never print an acceptance pass.

Deliverable: a small-motion test ready for supervised hardware use, with its exact
CLI and expected results documented after implementation. Do not publish placeholder
commands as if this diagnostic already exists.

### 7. Pass Offline and Simulation Release Gates

- Unit tests: message mapping/serialization, units, limits, optional dependencies,
  no non-arm commands, read-only protections, startup and disconnect behavior.
- Integration tests: real IK and simulator feedback, fake physical transport,
  stale/missing/out-of-order packets, invalid IK, lost tracking, clutch release,
  restart/re-engagement and failure during initialization or shutdown.
- Exercise the small-motion diagnostic in simulation using the same trajectory and
  checks, while labeling any hardware authority behavior that is only mocked.
- Re-run G1-29 bilateral VR simulation. Confirm the hardware opt-in cannot be
  reached accidentally through the simulation launcher or a default configuration.

Deliverable: pinned candidate revision, reproducible commands, reports and clear
pass/fail criteria. Passing these tests makes the tool ready for hardware review,
not physically validated.

### 8. Run the First Supervised Hardware Checks

After the operator approves the real setup, authority contract, support/stop
arrangement and operating bounds:

1. Run read-only feedback validation on the exact candidate revision.
2. Acquire arm authority at the measured pose with no requested displacement;
   acquisition itself can apply torque and is a motion-enabled test.
3. Run one bounded joint movement on one arm; verify identity, direction, tracking,
   other-joint behavior and the planned stop/handover. Stop on any discrepancy.
4. Repeat on the other arm only after reviewing the first result.

No universal angle, speed or torque values are preapproved here. Select those
against the actual hardware, attached payload, operating mode and operator review.
Hardware acceptance requires observation of the physical stop, not just exit code 0.

### After the First Hardware Gate

Continue with Stage 2 scripted small Cartesian movements, then Stage 3 one-arm and
bilateral VR. Add real-camera-to-headset delivery after physical arm control passes;
the first bounded joint tests do not require CloudXR or a camera. On headset video
loss, apply the declared control contract rather than allowing blind continuation
by accident. Finger control and mobile manipulation remain separate milestones.

No stage is authorized merely by the plan, passing simulation, or a read-only report.
Start physical control-authority design early, in parallel with simulation and upstream
work. Name the responsible operator, applicable manufacturer procedure, support and
stop arrangement, controller ownership, watchdog location, and reviewed operating limits.
The source-specific experimental preflight audit is not automatically valid for the
current development fork; re-audit its startup/reset/disconnect paths before motion.

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

Build `unitree_g1_lerobot/diagnostics/physical/verify_g1_ik_physical.py`, comparable in purpose to
[the MuJoCo diagnostic](../unitree_g1_lerobot/diagnostics/simulation/verify_g1_ik_mujoco.py), and run
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
