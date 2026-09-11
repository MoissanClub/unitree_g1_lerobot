# Three-Track Project Plan

This is the primary roadmap for `unitree_g1_lerobot`. The project has three work
tracks corresponding to the `robots/`, `xr/`, and `simulation/` package folders.
The existing [rung ladder](vr-teleop-g1-23-ladder.md) is a cross-track acceptance
sequence and historical record, not the source-code ownership structure.

## Development Workstreams

The upstream-oriented fork now has separate embodiment, Cartesian, simulation,
XR, XR-video, generic-hand, and BrainCo feature branches. See the
[branch stack verification](branch-stack-verification.md) for exact suites and
fresh-checkout merge reproduction. This contribution effort does not replace the
experimental three-script DDS workflow or its remaining acceptance work.

The [simulation and sim-to-real plan](sim-to-real-plan.md) records the 2026-09-11
split into simulation follow-ups and physical embodiment testing. Simulation follows
the existing non-physical handoff. Physical work progresses through read-only preflight,
bounded joint motion, scripted IK verification, and XR teleoperation on G1-29, then G1-23.
These workstreams span the three ownership tracks below and keep separate acceptance evidence.

## Status Summary

| Track | Implemented and Verified | Remaining |
|---|---|---|
| 1. G1-23 in LeRobot | Native embodiment/IK, sparse mapping, runtime gains/feedforward, shared G1 interface; both variants pass small-signal live and geometry suites | Broader workspace/near-limit acceptance, hardware support, contribution preparation |
| 2. XR for LeRobot | Both-arm bridge and real headless CloudXR/OpenXR/video for both variants; headset control and basic video reviewed | Failure/recovery, reconnect, and video timing/endurance acceptance |
| 3. G1 simulation | Live supported-arm backends; reviewed benchmarks; independent geometry, DDS baseline, camera capture and delivery tested | Larger trajectory/limit coverage, performance/endurance, native cleanup; matched models before waist control |

User visual review of the geometry, IK, and motor-comparison checkpoint is complete.
The structural refactor passed 25 project tests and 92 LeRobot G1 regression tests.
The native G1-23 live simulator is now implemented, with standalone and embedded
connection tests, both-arm DDS motion, feedback, and command-loss holding verified.
Full Rung 4 control acceptance, systematic video lifecycle/performance acceptance, and
physical-robot work remain pending. Camera capture and shared graphics/input submission
pass headless checks on both variants. The user verified video in the VR headset on
2026-09-10; the reviewed embodiment was not specified.

The XR extension passed headless three-launcher sessions for both embodiments and
the 31-test project suite with DDS enabled (30 passed, one GUI-only test skipped).
Repeated DDS sessions in one interpreter exposed a native callback teardown crash.
Fresh-process DDS tests contain it; responsibility among LeRobot, the Unitree SDK,
and CycloneDDS bindings is unresolved. Track 1/3 follow-up is an SDK-only reproducer
and an explicit-cleanup comparison, not an assumed CycloneDDS fix. See
[Finding 10](vr-teleop-g1-23-ladder.md#finding-10-dds-session-teardown).
See [native simulator operation and scope](g1-23-live-simulator.md).

## Track 1: G1-23 in the LeRobot Stack

**Owner:** `unitree_g1_lerobot/robots/`, plus a narrowly scoped LeRobot patch.

**Purpose:** make G1-29/G1-23 a configuration choice behind one LeRobot robot interface,
with the same method contract for teleoperation, simulation, and eventual hardware.
Different embodiments still expose different active joints and pose capabilities.

### Completed

- Native G1-23 joint/URDF/end-effector definitions and the existing parametric IK.
- Source-derived G1-29/G1-23 motor profiles using the same pinned-source methodology.
- Shared `UnitreeG1` implementation selected by `UnitreeG1Config.embodiment`.
- Variant-aware features, DDS-indexed defaults, state/command loops, reset/shutdown
  loops, and gravity-torque scattering. G1-29 defaults remain unchanged.
- G1-23 gravity adapter, configuration validation and serialization/factory tests.
- Reproducible LeRobot patch and safe, idempotent application helper.

### Remaining

1. Preserve the now-connected G1-23 backend and its basic standalone/embedded regression checks with Track 3.
2. Extend the passing per-joint DDS checks to near-limit motions and fault cases,
   retaining sparse-index, unused-slot, direction, and feedback assertions for both embodiments.
3. Preserve the verified runtime settings: LeRobot-default G1-29 gains, derived G1-23
   gains, and gravity compensation enabled. Characterize larger-motion tracking without
   treating live IK feedforward and measured-pose exact-model benchmark compensation
   as identical implementations. Gain comparison is a separate experiment.
4. Define measured-state initialization, command ownership, and stop/stale-command
   contracts with the simulator and later hardware adapter.
5. Validate physical G1-29 first when available, then G1-23. Recheck transport, gains,
   limits, modes, and operator stop procedures; no simulation safety assumption transfers
   automatically. If G1-29 hardware is unavailable, record the skipped baseline.
6. Prepare an upstream-focused contribution: review API/schema, dependency and asset
   packaging, source licenses, tests, and compatibility without adding local XR/simulator
   tooling as robot-class dependencies.

**Acceptance evidence:** interface/serialization tests, per-joint DDS tests, IK and
gravity ordering tests, G1-29 regressions, then separately recorded hardware evidence.
Do not implement another IK algorithm or another robot class to repeat completed work.

## Track 2: XR Support for LeRobot

**Owner:** `unitree_g1_lerobot/xr/` and XR-specific setup/runtime configuration.

**Purpose:** provide XR input and retargeting orchestration through LeRobot interfaces,
without encoding robot motor layouts or simulator internals in the input adapter.
The current integration uses Isaac Teleop/CloudXR; generality is an architectural aim,
not a claim that other providers or all LeRobot robots have been tested.

### Completed

- External CloudXR startup, controller smoke test, and existing XR-to-G1 bridge.
- CloudXR-only launcher has no robot diagnostic, embodiment selection, or LeRobot
  runtime dependency; it can start independently of the simulator and bridge.
- XR commands and simulator startup diagnostics default to gravity compensation ON;
  `--no-gravity-compensation` provides an explicit opt-out. DDS feedforward is sender-owned.
- Clutch/engagement, ready-pose behavior, diagnostics, and pose/button instrumentation.
- User-confirmed G1-29 and G1-23 right-controller engagement and movement in simulation.
- User-confirmed dual-arm headset visual review. This does not establish exhaustive
  tracking-loss, reconnect, or numerical accuracy coverage.
- Default dual-arm input (`--hand-side both`): one session, independent clutches,
  one bilateral IK solve and DDS command; headless simultaneous motion verified on both variants.
- Ergonomic service order: simulator, bridge, CloudXR, then put on/connect the headset.

### Remaining

1. Preserve the implemented embodiment-selectable bridge and three-launcher headless
   regression. Both variants have separate and simultaneous mock motion and real CloudXR/OpenXR evidence.
2. Record systematic startup diagnostic, engagement/release, invalid tracking, loss,
   and reconnect coverage on both variants beyond the completed visual review.
3. Retain G1-29 regression evidence and record bilateral/controller lifecycle coverage
   explicitly. Visual review is not exhaustive controller lifecycle acceptance.
4. Extend the basic user-reviewed mono headset display acceptance for Track 3 frames. The bridge
   shares one graphics/input OpenXR session; headless submission passes on both variants.
   Verify actual headset framing, reconnects, latency, and control-loop interference.

**Acceptance evidence:** input/frame-semantics and clutch tests, simulated-input tests,
observed headset control, and a separate observed camera-video test. A headset client
showing "Running" does not establish video delivery.

## Track 3: Simulation for G1

**Next-session priority:** extend the passing small-signal joint/IK/DDS and geometry
suites to broader motion coverage. Camera streaming and the verification organization
migration are complete. Follow the [non-physical plan](future-non-physical-work.md),
then lifecycle/recovery, performance/endurance, and native cleanup work.

**Owner:** `unitree_g1_lerobot/simulation/`. Benchmark orchestration and startup checks
are shared verification tooling under `diagnostics/`. Its `shared/`, `backends/`, and
domain-specific folders separate reusable calculations from execution; subsystem-based
`tests/` validates them. See [verification organization](verification-organization.md).

**Purpose:** provide native MuJoCo models, motor-driven simulation, rendering, and a
live command/state backend that the shared robot interface can use.

### Completed

- Working G1-29 embedded and standalone DDS simulator workflows and keyboard artifact.
- Separate native G1-29/G1-23 geometry and IK comparison viewers, reviewed by the user.
- Supported-arm motor benchmarks with gravity always enabled, compensation OFF/ON,
  four paired launchers and a synchronized 3-by-2 summary.
- Shared 34-case suite: holds, payloads, steps, sweeps, fast moves, and reversals;
  reports cover tracking, rise time, overshoot, settling, sag, and torque headroom.
- Real-viewer controls and per-panel motion checks. Preserve all verification launchers.
- Opt-in local robot-camera capture for both embodiments, isolated rendering process,
  bounded frame IPC, and independent preview; three-service headless camera matrix passes.

### Remaining

1. Extend the passing small-signal suites for both embodiments into broader motion
   acceptance. Native G1-23 reuses reviewed model/motor data with ten dynamic arm joints
   and supported pelvis/legs/waist, not a G1-29 visual substitute.
2. With Track 1, extend the tested motor-command-to-measured-state DDS path. Verify simulator
   ownership, timestep/control rate, contacts/support assumptions, limits, and stale
   command behavior. Do not silently turn the supported-arm benchmark into a claimed
   full-body/balance simulator.
3. Extend device-free Cartesian/orientation trajectories through IK -> DDS -> actuators.
   Compare target pose, commanded joints, measured joints, and measured FK. Separate
   kinematic residuals from actuator errors; establish numerical acceptance thresholds.
4. Preserve the implemented local-camera-to-XR-display interface. Local images, frame
   motion, age, and OpenXR submission are checked; quantify physics timing impact and
   extend basic headset presentation review to reconnect/endurance acceptance.
   See [camera checkpoint](camera-streaming.md).

**Acceptance evidence:** native models, measured motor-driven trajectories, DDS/timing
and command-loss tests, then camera-frame tests. Geometry/IK pose replay is not physics;
motor benchmark replay is not a live DDS/headset run.

## Shared Interfaces and Ownership

| Boundary | Provider | Consumer | Contract |
|---|---|---|---|
| Controller pose/buttons and validity | Track 2 XR input | Shared retargeting/control | Explicit frames, timestamps, validity, and engagement semantics |
| Embodiment/IK/joint capabilities | Track 1 robots | Tracks 2 and 3 | Selected model, active joints, limits, pose capabilities; no fake missing joints |
| Motor command and measured state | Track 1 robot/transport | Track 3 simulator or hardware | Sparse DDS layout, command ownership, timing, and stop behavior |
| Camera frames | Track 3 simulator or hardware cameras | Track 2 headset display | Camera identity, dimensions, timestamps, and lifecycle; details still to be decided |

`diagnostics/`, `tests/`, `assets/`, and `configs/` support these three tracks; they
are not a fourth product track. The table describes ownership and intended contracts,
not a claim that every boundary is fully implemented.

## Cross-Track Acceptance

Independent geometry verification is implemented as `--geometry` on the live suite.
The [source-family audit](g1-model-source-audit.md) corrects the initial torso-origin
comparison: neutral-waist arm geometry agrees in the common pelvis frame. Legacy versus
rev_1_0 waist kinematics differ under roll/pitch; use matched models before extending
waist control. Current arm-only checks are not full-body or physical calibration evidence.

The first automated joint and target-space live-path suite is now available as
`./run_verify_live_control.sh`; see [coverage and budgets](live-control-acceptance.md).
It does not close workspace-boundary, recovery, or sustained-performance acceptance.

Current future backlog and recommended experiments:
[Future non-physical work](future-non-physical-work.md). The user concluded this session
after migration commit `bbc08f4`; resume with broader motion acceptance, not camera
bring-up or a repeat refactor. The latest [handoff](rung4-handoff.md) supersedes older priorities.

1. **Next: Tracks 1 + 3.** Extend the passing baseline for both embodiments to broader
   workspace, near-limit, hold/reversal, bilateral, and orientation cases without a headset.
2. **Then Track 2 with Tracks 1 + 3.** Extend the completed headset visual review with
   systematic controller lifecycle checks and G1-29 regression.
   These checks contribute to **Rung 4 control** acceptance; broader numerical and
   sustained-operation evidence is still required.
3. **Tracks 3 + 2: camera feedback.** Local capture and submission are tested on both
   embodiments; basic headset video is user-verified. Reconnect/timing acceptance remains
   open for **Rung 4V**.
4. **Track 1 + Track 2: physical robots.** G1-29 baseline (**5a**), then G1-23 (**5b**),
   following the stage-specific gates in the [sim-to-real plan](sim-to-real-plan.md).
   The broader simulation backlog continues independently; relevant simulation evidence
   and physical preparation precede each hardware stage. Track 3 remains the regression
   environment. Future hand/tactile integration remains historical **Rung 6**.

The tracks can progress independently where their contracts are testable, but a
cross-track milestone requires evidence from the complete path. A new keyboard stage
is optional, not a prerequisite. No gain winner, maximum safe speed, or hardware
payload rating was established by the comparison benchmarks.

## References

- [Operator guide](operator-guide.md): setup, scripts, camera controls, headset sequence.
- [Architecture](architecture.md): shared G1 class, patch setup and upstream boundaries.
- [Motor comparisons](motor-config-comparison.md): test conditions, metrics, launchers.
- [Handoff](rung4-handoff.md): resume details and repository state at the checkpoint.
- [Rung ladder](vr-teleop-g1-23-ladder.md): historical milestones and acceptance detail.
