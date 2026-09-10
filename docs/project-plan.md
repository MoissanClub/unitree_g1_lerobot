# Three-Track Project Plan

This is the primary roadmap for `unitree_g1_lerobot`. The project has three work
tracks corresponding to the `robots/`, `xr/`, and `simulation/` package folders.
The existing [rung ladder](vr-teleop-g1-23-ladder.md) is a cross-track acceptance
sequence and historical record, not the source-code ownership structure.

## Status Summary

| Track | Implemented and Verified | Remaining |
|---|---|---|
| 1. G1-23 in LeRobot | Native embodiment/IK, sparse mapping, derived profiles, shared configurable G1 interface; reviewed geometry/IK and contract tests | Live command/feedback acceptance, runtime feedforward decision, hardware support and contribution preparation |
| 2. XR for LeRobot | CloudXR input and G1-29 right-controller-to-simulation motion | Embodiment-selectable bridge, G1-23 headset acceptance, broader lifecycle/bilateral testing, camera display in headset |
| 3. G1 simulation | Live G1-29 and native supported-arm G1-23 simulator; model/IK/motor comparisons and basic DDS/hold checks | Systematic DDS/actuator timing and trajectory acceptance, camera capture |

User visual review of the geometry, IK, and motor-comparison checkpoint is complete.
The structural refactor passed 25 project tests and 92 LeRobot G1 regression tests.
The native G1-23 live simulator is now implemented, with standalone and embedded
connection tests, both-arm DDS motion, feedback, and command-loss holding verified.
Full Rung 4 control acceptance, camera streaming, and physical-robot work remain pending.
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
2. Verify each of the ten active arm joints through actual DDS command and feedback,
   including sparse indices, unused slots, signs, ranges, and partial commands.
3. Explicitly choose live gain/feedforward settings and verify them. The current live
   gravity path uses command-pose IK gravity; the benchmark uses measured-pose,
   exact-model gravity. Do not treat them as already-equivalent implementations.
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
- Clutch/engagement, ready-pose behavior, diagnostics, and pose/button instrumentation.
- User-confirmed G1-29 right-controller engagement and movement in simulation.
- Ergonomic service order: simulator, bridge, CloudXR, then put on/connect the headset.

### Remaining

1. Bind the bridge to Track 1's selected embodiment instead of its current G1-29-specific
   IK/joint imports. Keep controller acquisition and clutch semantics shared.
2. Verify scripted/mock input without a headset, then G1-23 headset operation with
   startup diagnostics, engagement/release, invalid tracking, loss, and reconnects.
3. Retain G1-29 regression evidence and record bilateral/controller lifecycle coverage
   explicitly. Right-controller success is not exhaustive bilateral acceptance.
4. After control acceptance, own the headset display/streaming integration for robot
   camera frames supplied by Track 3 (and later hardware). Decide mono/stereo and view
   behavior; verify framing, reconnects, frame age, and control-loop interference.

**Acceptance evidence:** input/frame-semantics and clutch tests, simulated-input tests,
observed headset control, and a separate observed camera-video test. A headset client
showing "Running" does not establish video delivery.

## Track 3: Simulation for G1

**Owner:** `unitree_g1_lerobot/simulation/`. Benchmark orchestration and startup checks
are shared verification tooling under `diagnostics/`.

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

### Remaining

1. Extend the implemented native G1-23 live factory's smoke checks into systematic
   acceptance. It reuses the reviewed model/motor data with ten dynamic arm joints
   and supported pelvis/legs/waist, not a G1-29 visual substitute.
2. With Track 1, test motor-command-to-measured-state DDS round trips. Define simulator
   ownership, timestep/control rate, contacts/support assumptions, limits, and stale
   command behavior. Do not silently turn the supported-arm benchmark into a claimed
   full-body/balance simulator.
3. Run device-free Cartesian/orientation trajectories through IK -> DDS -> actuators.
   Compare target pose, commanded joints, measured joints, and measured FK. Separate
   kinematic residuals from actuator errors; establish numerical acceptance thresholds.
4. Supply live camera frames to Track 2 after control acceptance. Verify local images
   first and measure rendering cost, frame timestamps, and simulation timing.

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

1. **Next: Tracks 1 + 3.** With the live G1-23 backend in place, exercise individual DDS joints,
   then run scripted IK-to-actuator tests without a headset.
2. **Then Track 2 with Tracks 1 + 3.** Verify G1-23 headset control and G1-29 regression.
   This completes the remaining historical **Rung 4 control** acceptance.
3. **Tracks 3 + 2: camera feedback.** Local capture, headset delivery, reconnect/timing
   tests. This is **Rung 4V**, currently unimplemented/unverified.
4. **Track 1 + Track 2: physical robots.** G1-29 baseline when available (**5a**), then
   G1-23 (**5b**), after simulation and video acceptance. Track 3 remains the regression
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
