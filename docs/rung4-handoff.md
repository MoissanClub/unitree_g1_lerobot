# Rung 4 Handoff

## Resumed: Live Numerical Acceptance

The first non-physical acceptance suite is implemented in `run_verify_live_control.sh`.
It covers bidirectional per-joint steps and small Cartesian/orientation sweeps through
the actual simulator/LeRobot DDS path. See [live acceptance](live-control-acceptance.md)
for thresholds, reports, and remaining coverage. This supersedes the end-of-day stop
below; do not equate the new smoke suite with full non-physical acceptance.

## Workstream update: 2026-09-11

Continue simulation follow-ups using [future non-physical work](future-non-physical-work.md).
The separate [sim-to-real plan](sim-to-real-plan.md) records physical preflight, simple
joint motion, scripted IK verification, and XR teleoperation on G1-29, then G1-23.
The earlier simulation-only session boundaries below remain historical context.

## End-of-Day Handoff: 2026-09-10

Camera streaming is implemented and the user verified video with the VR headset.
Implementation `4612a5c` is committed and pushed. Today's work is concluded.
Resume with [future non-physical work](future-non-physical-work.md): systematic joint DDS
and numerical IK-to-physics acceptance first, then recovery, performance, and native cleanup
investigations. Do not rebuild the working camera path or start physical actuation.
The camera section below records the earlier implementation sequence, not the current backlog.

## Next Session: Camera Streaming

**Local camera checkpoint implemented:** simulator `--camera` now publishes a torso-mounted
mono RGB8 stream on both embodiments; `./view_g1_camera.sh` previews it independently.
The renderer runs in a child process on copied state, with bounded nonblocking IPC.
The three-launcher headless camera matrix passed on both variants, and saved images
were inspected for both hands and correct orientation. See [camera checkpoint](camera-streaming.md).
SDK XR image submission/shared-session integration is now implemented: add `--video`
to the external-simulator bridge. The user verified video in the VR headset on 2026-09-10.
Next is systematic reconnect and performance acceptance,
not rebuilding camera capture. See the camera document for the complete three-command sequence.
The original sequence below remains context; local capture/frame handoff are now implemented.

**Earlier user direction (now implemented):** after dual-arm headset visual review, add
robot-camera streaming to the headset (Rung 4V), not another model/IK/motor comparison or backend bring-up.
Systematic numerical DDS/actuator and controller-lifecycle acceptance remains open, but
is deferred rather than a prerequisite to beginning camera work. Do not mark it complete.

**Baseline:** `8d29a6f` on `main` was committed and pushed. It enables gravity compensation
by default and makes CloudXR robot-independent. The latest headset-review and handoff
documentation updates follow that implementation baseline; check `git status` before continuing.
Latest verification: 38 tests passed across the unit and isolated DDS runs; one GUI test
skipped. Actual three-launcher headless checks passed for both embodiments, including
dual-arm motion and clean shutdown. Logs: `/tmp/g1-xr-headless-vt4dfxg1` (temporary).
CloudXR also started and stopped successfully without a simulator or bridge.

### Working Operator Sequence

From `~/lerobot-sim/unitree_g1_lerobot`, in separate terminals:

```bash
# Simulator; use ssh -Y for its local Tk view.
./run_g1_mujoco_dds_sim.sh --embodiment g1_23

# XR bridge; both arms and gravity compensation are enabled by default.
./run_xr_g1_mujoco.sh --embodiment g1_23 --external-g1-sim --external-cloudxr --wait-for-cloudxr --no-wait

# CloudXR only: no embodiment, robot diagnostic, or confirmation prompt.
./run_isaac_teleop.sh
```

Use `g1_29` on the simulator and bridge for that embodiment. Confirm the simulator and
bridge startup motions, then connect the headset. CloudXR may start independently in
either order. `--no-wait` skips the headset-attachment prompt, not the bridge diagnostic
confirmation; `--headless` suppresses viewers/confirmations for automated checks.

### Implementation Sequence

1. Inspect the installed Isaac Teleop/CloudXR SDK and its rendering examples to establish
   the supported image-submission path. The current bridge opens a headless OpenXR
   session for controller input; determine how rendering and input can share its session
   before adding another XR client. Do not assume the existing browser's "Running" screen
   can display arbitrary camera frames or that a second session can coexist safely.
2. Define a camera in each simulator, render actual robot-camera frames, and verify local
   images first. Existing Tk views are workstation previews, not delivered headset video.
   Keep the robot arms and useful workspace visible. Start with one embodiment, then
   verify the same frame contract on the other.
3. Add a bounded latest-frame handoff with camera identity, dimensions, pixel format,
   sequence number, and timestamp. Keep capture under `simulation/`, transport/display
   under `xr/`, and robot/IK code independent. Avoid blocking the physics or DDS loop on
   rendering, encoding, or a disconnected headset. Use supported SDK transport where available.
4. Submit frames using the verified XR integration while preserving controller input,
   independent clutches, and the single robot-command publisher. Preserve no-video mode
   and all existing verification launchers; do not reintroduce robot settings into the
   CloudXR-only service launcher.
5. Add local/headless frame checks, then ask for headset review only when images actually
   reach the display. Verify both embodiments, correct framing/orientation, visibly live
   motion, reconnect behavior, and continued bilateral control. Measure frame rate,
   frame age, and physics/control timing with video off versus on.

**Implemented checkpoint:** fixed torso-mounted mono camera, 640x480 at up to 20 FPS,
same-host bounded frame IPC, and a head-following monitor via SDK VizSession. Its graphics
handles are shared with controller input in an isolated worker. The original sequence
above records the implementation plan; capture and submission are now complete.
Stereo/head-coupled optics and numerical acceptance targets remain future decisions. Keep physical
robot work out of this session. G1-23 remains supported-arm simulation, not full-body balance.

### Files and Environment

- `simulation/g1_mujoco_dds_sim.py`: standalone selection, Hub rendering, and Tk viewer.
- `simulation/native_g1.py`, `simulation/native_g1_viewer.py`: native G1-23 physics/viewer.
- `xr/xr_to_g1_mujoco.py`, `xr/both_controllers.py`: XR session/input and bilateral control.
- `xr/cloudxr_session.py`, `run_isaac_teleop.sh`: robot-independent CloudXR service lifetime.
- `diagnostics/verify_xr_headless.py`: preserve the real three-launcher regression matrix.
- Package paths above are under `unitree_g1_lerobot/`. Use descriptive filenames, not rung numbers.
- Robot Python: `/home/dwei/miniforge3/envs/lerobot-g1/bin/python`.
- XR Python/SDK: `/home/dwei/.venvs/isaacteleop/bin/python` and its Python 3.12 site-packages.
- Adjacent LeRobot: `/home/dwei/lerobot-sim/lerobot`; preserve its existing changes and patch workflow.

Headset image submission is implemented; **actual headset video is user-verified**.
Local capture and headless graphics/input submission are verified.
Keep the DDS native teardown
finding open; use fresh processes for independent sessions, avoid concurrent test publishers,
and do not claim headset video success based only on CloudXR service readiness.

## Resume Point

Resume using the [three-track project plan](project-plan.md): Track 1 owns G1-23
LeRobot support (`robots/`), Track 2 owns XR (`xr/`), and Track 3 owns G1 simulation
(`simulation/`). Track 3's native supported-arm G1-23 backend now runs, including
basic standalone/embedded DDS checks. Systematic Track 1 joint/IK/DDS
acceptance and Track 2 lifecycle testing remain deferred work. Dual-arm headset visual review
is complete. Existing rung names remain shared
acceptance gates, not separate implementation tracks.

The simulator and bridge accept `--embodiment`; all three accept `--headless`. Both variants passed
real CloudXR/OpenXR startup and separate plus simultaneous mock left/right motion with
measured feedback. The user confirmed right-arm headset control on both embodiments.
The bridge now defaults to `--hand-side both`, with independent clutches in one XR
session and one combined IK/DDS update. The user has now completed dual-arm headset
visual review. Do not infer exhaustive release, tracking-loss, reconnect, or numerical
acceptance from that confirmation. Single-hand modes remain available explicitly.

The XR launcher and simulator startup diagnostics now default to gravity compensation
ON for both embodiments, with `--no-gravity-compensation` as an explicit opt-out.
DDS feedforward remains sender-owned, never added twice by the simulator. Native
G1-23 idle/stale hold retains its compensated safety behavior. CloudXR is now independent:
no lowering diagnostic, embodiment/gravity parameter, confirmation prompt, or LeRobot
checkout/G1 conda dependency. Start it with `./run_isaac_teleop.sh`, in either service order.

Package modules now use descriptive names: `xr.xr_to_g1_mujoco`,
`diagnostics.verify_g1_ik`, and `diagnostics.verify_g1_ik_mujoco`. Root shell launchers
are unchanged. The IK comparison motion profile is `cartesian_orientation_sweep`.
Do not restore planning-stage numbers in package filenames, comments, or identifiers.

Dual-arm headless verification logs: `/tmp/g1-xr-headless-29tjmzcw` (local, temporary).
Both-arm runs measured peak-to-peak joint motion of 0.460/0.505 rad (left/right) for
G1-29 and 0.476/0.509 rad for G1-23. Separate single-hand regressions, actual dual-stream
OpenXR startup, mismatched-embodiment rejection, and clean shutdown also passed.
These are motion-presence checks, not Cartesian accuracy or hardware-safety acceptance.
Unit tests additionally cover per-hand release/tracking loss, invalid poses/buttons,
re-engagement origins, orientation targets, non-identity joint ordering, invalid IK
results, and reading both controller streams in one session step.

**Open finding:** repeated DDS sessions in one interpreter can crash in a native
publication-matched callback. LeRobot's missing explicit channel cleanup and the SDK's
listener lifecycle are investigation leads; responsibility is not yet isolated.
Fresh-process tests are containment, not a root-cause fix. Continue with the SDK-only
reproducer described in [Finding 10](vr-teleop-g1-23-ladder.md#finding-10-dds-session-teardown).
The separate G1-29 missing-image-publisher shutdown guard is fixed in the LeRobot patch.

The configurable runtime structure has now been implemented and tested after the
benchmark review. See [architecture](architecture.md#configurable-g1-runtime-structure).
Use `unitree_g1_lerobot.robots.unitree_g1` to register G1-23 and import the shared
LeRobot classes. The LeRobot checkout changes are reproduced by
`apply_lerobot_embodiment_patch.sh` and the versioned patch; there are no variant robot
subclasses. Configuration, active features, sparse DDS mapping, motor defaults, and
IK/gravity selection are implemented. G1-23 `connect()` now creates the native
supported-arm backend; hardware remains blocked. See [live simulator](g1-23-live-simulator.md)
for the tested standalone launcher, confirmation flow, and remaining acceptance boundary.

The user has confirmed visual review of the motor-comparison checkpoint. Geometry,
scripted IK, and the supported-arm motor benchmarks are reviewed. **Rung 4 is not
complete:** systematic per-joint, Cartesian/DDS, and XR lifecycle acceptance remains open.
The next session prioritizes camera streaming as directed above.
Do not repeat the completed comparisons or backend bring-up as a new milestone.

The live-simulator baseline is `252e29b` on `main`; this headless XR extension is
subsequent work. Remote: `git@github.com:MoissanClub/unitree_g1_lerobot.git`.
Check both repository worktrees before resuming; adjacent LeRobot edits are distributed
through this repository's versioned patch rather than pushed to the LeRobot remote.
Project directory:
`~/lerobot-sim/unitree_g1_lerobot`; adjacent LeRobot checkout: `../lerobot`.
Python environment: `~/miniforge3/envs/lerobot-g1/bin/python`.

## Completed Artifacts

- Preserve `run_compare_g1_29_g1_23_urdf_mesh.sh` and
  `run_compare_g1_29_g1_23_ik.sh` as independent geometry and kinematics checks.
- Source-derived profiles: `configs/motors/g1_29.json` and `g1_23.json`.
  Derivation and pinned upstream provenance live in
  `unitree_g1_lerobot/robots/motor_configs.py` and `assets/g1/motor_sources/`.
- Physics benchmark: `unitree_g1_lerobot/simulation/motor_bench.py`.
- Shared 34-case suite: `unitree_g1_lerobot/diagnostics/motor_suite.py`.
- Reports and Tk replay: `unitree_g1_lerobot/diagnostics/compare_motor_configs.py`.

From an `ssh -Y` terminal, run one launcher at a time:

```bash
cd ~/lerobot-sim/unitree_g1_lerobot
./run_compare_g1_29_motor_configs_no_gravity_compensation.sh
./run_compare_g1_29_motor_configs_with_gravity_compensation.sh
./run_compare_g1_29_g1_23_motor_configs_no_gravity_compensation.sh
./run_compare_g1_29_g1_23_motor_configs_with_gravity_compensation.sh
./compare_motor_config.sh
```

The summary columns are LeRobot G1-29, derived G1-29, and derived G1-23; rows are
compensation OFF above and ON below. All panels replay the same test time.
All five launchers compute physics and reports **before** opening the Tk viewer.
The user reported a missing summary window before later confirming review. An
early-window/progress implementation was proposed but interrupted and never applied;
do not assume it exists. If revisited, distinguish startup computation delay from
an X-display failure. `--tests pitch_20deg_step extended_hold_1kg` shortens startup.

## What Was Verified

All four pair launchers completed 34 scenarios and opened their viewers. The summary
completed 204 trials. Reports had the expected distinct traces and 34 previews.
Sixteen automated tests passed with real Tk/EGL checks enabled, including navigation,
pause/resume, restart, panel ordering, and robot-pixel motion in every panel.

```bash
G1_TEST_VIEWER=1 ~/miniforge3/envs/lerobot-g1/bin/python -m unittest discover -s tests -v
```

Without `G1_TEST_VIEWER=1`, only the real-viewer interaction test is skipped.
Results normally go under ignored `artifacts/motors/`; do not commit generated runs.
See [benchmark details](motor-config-comparison.md) for metrics and test conditions.

## Interpretation and Limits

- All runs simulate gravity. Compensated runs add exact-model gravity torque at
  measured joint positions with zero velocity in a separate MuJoCo data object.
  Known payload mass is included; no Coriolis, acceleration, or friction feedforward.
  Total PD-plus-feedforward torque is clipped to motor limits.
- Derived G1-29 shoulder pitch/roll gains are Kp=80 versus LeRobot Kp=50, both Kd=3.
  Other arm gains match. No final runtime gain choice or universal winner was approved.
- Faster steps/sweeps, short point-to-point moves, and reversals expose response-time,
  overshoot, and saturation tradeoffs. These are not maximum safe speed ratings.
- Extended holds use 0, 0.25, 0.5, and 1 kg per hand. The final pose separates the
  hands and is collision-free on both models. A narrower earlier pose caused G1-23
  hand collisions and was corrected. These loads are not hardware payload ratings.
- Pelvis, legs, and waist are supported. Comparisons use deterministic joint targets,
  not live IK/DDS, and playback replays measured physics trajectories.
- Cross-embodiment common-joint targets do not imply identical Cartesian hand paths.
  The five-joint arm cannot independently satisfy all six Cartesian pose objectives.

## Remaining Rung 4 Work

1. Preserve the completed native G1-23 runtime integration and basic motor-driven
   motion/feedback checks. Startup and stale-command holding use exact-model gravity
   compensation; active DDS control uses the sender's gains and feedforward without
   extra compensation. See [live simulator scope and verification](g1-23-live-simulator.md).
   The G1-29 launcher/viewer regression passed with its startup diagnostic skipped;
   broader G1-29 motion and headset regression remains pending.
2. Exercise each of ten active arm joints through a separate DDS sender. Verify
   names, sparse transport indices, directions, ranges, unused slots, and feedback.
3. Run deterministic Cartesian/orientation targets through IK -> DDS -> actuators.
   Measure IK residuals separately from actuator tracking error. Define acceptance
   thresholds and verify control timing, command stop, and stale-command behavior.
4. The XR bridge now selects either embodiment; all three launchers support headless
   mode. Mock left/right motion and real CloudXR/OpenXR sessions pass for both variants.
   Verify actual headset startup diagnostics, engagement,
   release, tracking loss/disconnect, and headset control. Retain G1-29 regression
   evidence and document G1-23 orientation limitations.

No mandatory new keyboard stage: scripted DDS tests isolate integration variables.
Preserve existing G1-29 keyboard, DDS, and XR launchers. Keep robot support, simulation,
diagnostics, and CloudXR responsibilities separate for possible LeRobot contribution.
The user prefers continued implementation with minimal intervention; ask only when
a real tradeoff or required external observation blocks progress.

Robot-camera submission is implemented; in-headset acceptance remains open **Rung 4V**,
and is now the next-session priority following visual headset review. Physical G1-29 baseline is
Rung 5a, followed by G1-23 work. A connected headset showing "Running" is not evidence
of camera streaming. Refer to the [ladder](vr-teleop-g1-23-ladder.md) for acceptance.
