# Robot Camera and Headset Video

Local camera capture and OpenXR image submission are implemented for both G1-29 and G1-23.
The user verified video in the VR headset on 2026-09-10. This confirmation does not
specify an embodiment or establish exhaustive reconnect, latency, or lifecycle acceptance.
The camera is an illustrative torso-mounted head view, not a
calibrated model of a physical G1 camera. Both arms/hands are visible in reviewed frames.

## Run

In the simulator terminal, add `--camera` to the existing command:

```bash
./run_g1_mujoco_dds_sim.sh --embodiment g1_23 --camera
```

In a separate `ssh -Y` terminal, preview the actual published camera stream:

```bash
./view_g1_camera.sh
```

The original simulator Tk window remains a spectator view. The new preview shows the
same robot-camera image as the XR display. In the bridge terminal, enable video:

```bash
./run_xr_g1_mujoco.sh --embodiment g1_23 --external-g1-sim --external-cloudxr --wait-for-cloudxr --no-wait --video
```

Then start `./run_isaac_teleop.sh` in the third terminal and connect the headset as usual.
The bridge waits for CloudXR. Confirm each startup motion before steady-state control.
The headset should show a mono virtual monitor following your head, with both robot arms
visible. The robot camera itself stays torso-mounted; looking around does not steer it.
Both controllers still operate their independent clutches. Omit `--video` for input-only mode.
For unattended operation add `--headless` to all three commands; this disables Tk/prompts,
not EGL rendering or OpenXR video. A GPU and the installed Isaac Teleop Viz/CUDA stack are required.
Use `g1_29` on the simulator/bridge for that embodiment; the preview has no embodiment option.

For a noninteractive camera consumer:

```bash
./view_g1_camera.sh --headless --duration-s 10 --save-frame /tmp/robot-camera.png --report /tmp/robot-camera.json
```

`--headless --camera` on the simulator publishes frames without any Tk/X window or
confirmation prompt. `--camera` is opt-in; omitting it preserves the no-camera workflow.

## Camera Settings

- `--camera-width 640 --camera-height 480`: RGB8 output dimensions.
- `--camera-fps 20`: capture scheduling limit, independent of physics/control rate.
- `--camera-pitch 35`: downward pitch relative to the torso, in degrees.
- `--camera-fovy 90`: vertical field of view in degrees.
- `--camera-channel PATH`: local frame channel; give the same path to the preview and bridge.
- Bridge: `--video-max-age-s 0.5`, `--video-screen-distance 1.5` metres,
  `--video-screen-width 1.8` metres. Missing, stale, or wrong-embodiment images show
  a placeholder rather than a frozen live-looking image.
- Default camera ID: `robot_head`; mount: `torso_link`, offset `[0.10, 0, 0.38]` metres.
  Optical forward is robot +X pitched down; up is +Z pitched forward. Body rotation
  rotates the complete camera pose, including roll. No spectator camera settings are reused.

Defaults are shared by both embodiments. These are inspection-camera parameters, not
physical sensor calibration, stereo settings, or head-coupled viewing.

## Process and Frame Contract

The simulator launcher starts a child renderer process with a binary copy of the live
MuJoCo model and private `MjData`/EGL resources. After a physics step, the simulator
offers copied qpos, mocap state, simulation time, and a host monotonic capture timestamp
to a size-one queue. Busy queues drop offers. Rendering never runs under the physics
lock and a failed renderer disables video without stopping control.

The renderer publishes a memory-mapped latest-frame file, defaulting to
`/tmp/lerobot-camera-UID.rgb`. It is same-host IPC, not network DDS image transport.
The file contains a 4 KiB JSON header region and one fixed-size packed RGB8 image.
Metadata includes protocol version, camera/embodiment identity, producer session ID,
sequence, dimensions, simulation time, capture/publish timestamps, render duration,
mount configuration, and world-space optical pose.

Nonblocking file locks prevent torn reads; busy consumers/producers skip a frame.
A separate `.owner` lock prevents competing producers. Restart replaces the data inode,
so old readers cannot be invalidated by truncation. Consumers reopen each read, reject
frames older than one second by capture timestamp, and identify restarts by session ID.
Normal shutdown removes the data channel; the empty owner-lock file is intentionally
retained. Rendering is bounded by one active snapshot plus one queued snapshot, not an
unbounded backlog. CPU readback/copies are deliberate in this first checkpoint, not a
claim of zero-copy GPU transport.

Code ownership:
- `simulation/robot_camera.py`: mounted optics, isolated rendering, state offers/lifetime.
- `simulation/camera_frames.py`: dependency-light frame protocol; no MuJoCo/DDS/XR imports.
- `diagnostics/simulation/view_robot_camera.py`: independent local reader and preview.
- `xr/camera_display.py`: CUDA upload and SDK `VizSession` mono quad presentation.
- `xr/video_controller.py`: isolated graphics/input worker sharing one OpenXR session.
  The parent IK/DDS loop reads a bounded mailbox; input older than 250 ms releases clutches.
  Session exit can reconnect with `--wait-for-cloudxr`. Worker shutdown has a bounded join
  and terminate fallback for blocked runtime calls. CloudXR service remains robot-independent.

## Verification

From an idle simulator/CloudXR setup:

```bash
conda run --no-capture-output -n lerobot-g1 python -m unitree_g1_lerobot.diagnostics.xr.verify_xr_headless --video
```

This runs the real simulator, XR bridge, and CloudXR launchers with `--headless` on
both embodiments, plus a separate camera consumer during mock bilateral movement.
It checks fresh/nonblank/changing pixels, increasing sequence and simulation timestamps,
matching embodiment, measured arm movement, and clean channel/process shutdown. Real
OpenXR graphics/input session creation, render requests, and camera uploads are checked,
not actual headset reception. Images and JSON
reports are written to the printed temporary log directory.

Observed initial matrix (`/tmp/g1-xr-headless-u992e0b2`): G1-29 18.7 FPS and G1-23
18.6 FPS; p95 capture-to-reader age about 12 ms on each. This is local frame age,
not headset latency. Acceptance guards are at least 30 frames, 5 FPS, p95 age below
500 ms, nonblank images and visible changes. These broad smoke thresholds are not
final performance targets. Numerical physics timing A/B acceptance remains separate.

The initial suite with DDS and EGL camera checks enabled passed 45 of 47 tests;
the GPU display test and GUI motor-viewer test were skipped in that run. Both display/mailbox
tests passed separately with `G1_TEST_VIDEO=1`. The actual Tk camera preview was separately run
under Xvfb and its screenshot inspected. Those automated checks alone make no headset
video claim; the later user review is recorded below.

Protocol tests cover contention, staleness, shape checks, exclusive ownership, and
restart with an existing mapping. Enable `G1_TEST_CAMERA=1 MUJOCO_GL=egl` for real-EGL
tests of optical pose/orientation and renderer-failure isolation; `G1_TEST_DDS=1`
enables the existing fresh-process DDS cases.

## Headset Review

**Latest end-of-day status:** delivery remains implemented and basic headset video
user-reviewed. Migration `bbc08f4` reverified both real headless XR/video matrices and
the separate GPU display checks. See [migration evidence](verification-organization.md#migration-verification-2026-09-10).
The next project task is broader motion acceptance; camera follow-ups are recovery,
video off/on performance, endurance, and native OpenXR cleanup, as ordered in the
[non-physical backlog](future-non-physical-work.md). Do not restart camera bring-up.

**User verification, 2026-09-10:** video was verified with the VR headset. Basic visible
delivery is confirmed; the systematic checks below remain a follow-up checklist.

Headless video matrix passed for both embodiments in `/tmp/g1-xr-headless-3hc8owod`:
real OpenXR sessions plus right, left, and bilateral mock control all submitted video.
Each mock run uploaded 192-196 fresh camera frames; both cameras averaged 18.8 FPS
with local p95 frame ages of 11-12 ms. These are not network/headset latency measurements.

Verify live arm motion in the monitor, correct left/right and up/down orientation,
bilateral control while viewing, and headset disconnect/reconnect. Restarting the camera
producer should recover from the placeholder. The offscreen GPU test checks color and
orientation, stale/source rejection, and producer restart; it is not headset evidence.
Enable `G1_TEST_VIDEO=1` with the Isaac Teleop site-packages available to run that test.
End-to-end headset latency and comparative control timing remain unmeasured.
The installed SDK currently logs `XR_ERROR_SESSION_NOT_STOPPING` during graphics teardown
even on successful exit; this SDK cleanup warning is not treated as a passed lifecycle test.
