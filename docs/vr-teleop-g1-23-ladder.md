# VR Teleoperation of the Unitree G1-23 on LeRobot — Validation Ladder

Working method and status log. September 2026.

Current checkpoint (2026-09-09): G1-29 controller-to-simulation teleoperation was
confirmed by the user. G1-23 geometry and scripted IK have been visually reviewed;
live G1-23 physics/DDS integration is still pending. Robot camera video has not been
streamed to or verified in the headset. The code now lives in responsibility-based
subpackages; see [architecture](architecture.md).

**Goal:** drive a Unitree **G1-23** (5-DoF arms) from a VR headset through the **LeRobot**
stack, and contribute the result upstream.

**Method:** never change more than one variable at a time. Every rung starts from a
configuration already known to work and alters exactly one thing, so a failure has exactly
one candidate cause.

---

## Why this goal

| Choice | Reason |
|---|---|
| **LeRobot** | Owns the dataset format and now has first-class G1 support (π0/π0.5, RTC, MuJoCo sim, 29 and 23 DoF robot classes). It is the gravitational centre. |
| **Isaac Teleop** for XR | Already an accepted, documented LeRobot dependency with an `XRController` `Teleoperator` subclass. Adding a *robot target* to an existing device beats proposing a new device abstraction. |
| **G1-23** | Extend the tested G1-29 XR/IK workflow to five-joint arms. LeRobot documents both variants; this checkout's main IK/control path is G1-29-specific. |
| **Not forking `xr_teleoperate`** | Vendor sample code — no tests, no CI, 200 lines duplicated seven times. Good for bug fixes, hostile to architecture. |

### What already exists vs. what must be built

```
Isaac Teleop XR  →  LeRobot  →  SO-101          ✅ ships today
Isaac Teleop XR  →  Isaac ROS/Jetson  →  G1     ✅ ships today (NVIDIA's own stack, not LeRobot)
Isaac Teleop XR  →  LeRobot  →  G1-29 sim       ✅ user-confirmed controller input
Isaac Teleop XR  →  LeRobot  →  G1-23 sim       ⬜ live integration pending
Robot camera    →  VR headset                  ⬜ not implemented/verified
```

The current integration uses Isaac Teleop as an **input source**: it receives controller
poses through CloudXR. It does not implement the return path from robot camera frames
to a headset display. Whether the control target is MuJoCo or real hardware is a separate
concern from the input binding.
`XRController.get_action()` returns `{grip_pos, grip_quat, squeeze, trigger}` already
rebased into the robot frame. The interface between input and retargeting is a **4×4
homogeneous wrist pose** — the same seam `xr_teleoperate` uses with a completely different
input device.

---

## The ladder

| # | Change | Proves | Devices | Status |
|---|---|---|---|---|
| **0** | CloudXR + headset, NVIDIA's own example | The headset works at all | headset | ✅ input connection proven in rung 3; full standalone checklist not recorded |
| **1** | Shipped SO-101 example | Isaac Teleop → LeRobot path | + SO-101 | ⏭️ skipped |
| **1'** | Same headset via LeRobot's `XRController` | The LeRobot binding, minus actuators | headset | ✅ integrated input path proven in rung 3 |
| **2a** | Script → `G1_29_ArmIK` | IK stack, no sim, no XR | **none** | ✅ done |
| **2b** | 2a → MuJoCo | Robot interface + sim | **none** | ✅ done |
| **3** | Join 1' + 2b | **The glue — the actual contribution** | headset | ✅ done |
| **4** | G1-29 sim → G1-23 sim | Embodiment, physics, DDS, XR control | none first, then headset | 🟡 visual review complete; live runtime pending |
| **4V** | Robot camera → VR headset | Live visual feedback alongside control | headset | ⬜ planned after rung 4 control acceptance |
| **5a** | G1-29 sim → real G1-29 | Hardware transport and control baseline | + G1-29 | ⬜ planned first if hardware is available |
| **5b** | G1-23 sim → real G1-23 | Five-joint embodiment on hardware | + G1-23 | ⬜ after hardware baseline |
| **6** | Gripper → BrainCo hand | Tactile integration | + BrainCo | ⬜ todo |

Rungs 1' and 2 are independent — one needs the headset, the other needs nothing — so they
can run in parallel or in either order.

---

## Rung detail

### Rung 0 — CloudXR + headset, no robot
Run NVIDIA's `gripper_retargeting_example_simple.py`. See [Rung 0 install notes](Rung0_isaac-teleop-install-notes.md).

**Acceptance:** session stable 5+ min · both controllers tracked · squeeze/trigger sweep
0→1 · poses continuous · tracking survives torso rotation and full arm extension.

**Status:** CloudXR/headset controller input was subsequently verified in the G1-29
Rung 3 workflow. The complete standalone endurance checklist above has not been recorded.
The original headset compatibility uncertainty no longer blocks that tested input path;
this does not establish robot-camera streaming or support for every headset model.

### Rung 1 — SO-101 (skipped)
Needs a physical SO-101 (~$150). Skipping it removes an isolation point: rung 3 then has
two unvalidated halves instead of one. An SO-101 is also the reference platform maintainers
test against — worth owning if you intend to validate PRs.

### Rung 1' — XR teleoperator alone (the cheap substitute)
Recovers most of rung 1 for ten lines and no purchase:

```python
from examples.isaac_teleop_to_so101.isaac_teleop.config_isaac_teleop import XRControllerConfig
from examples.isaac_teleop_to_so101.isaac_teleop.teleop_xr_controller import XRController

teleop = XRController(XRControllerConfig())
teleop.connect()
for _ in range(500):
    a = teleop.get_action()
    print(a["grip_pos"], a["squeeze"])
```

Run from the LeRobot repo root. Validates the session lifecycle, the device, and
`get_action()` — everything except actuators. What it prints is exactly what rung 3's glue
will consume.

### Rung 2 — split, because it has two failure domains
- **2a** IK only: Hub download, conda Pinocchio + CasADi, IPOPT, residuals, joint ordering.
- **2b** + MuJoCo: `make_env`, robot interface, DDS on loopback, action/observation round trip.

Zero devices. Seven install blockers surfaced here — see
[Rung 2 install notes](Rung2_lerobot-g1-mujoco-install-notes.md). **Requires a real X session or `xvfb-run`.**

### Rung 3 — the contribution
`grip_pos`/`grip_quat` → 4×4 → `solve_ik` → 29-joint action dict → `UnitreeG1(is_simulation=True)`.

Both halves proven, so any failure is in ~20 lines of glue.

**Current status:** the user confirmed right-controller engagement and movement driving
the G1-29 simulation. The bridge uses `G1_29_ArmIK` with conda-forge Pinocchio/CasADi and
the existing clutch implementation; the default engagement input is max(squeeze, trigger).
It runs in `lerobot-g1` and appends the Isaac Teleop environment for XR imports. CloudXR
runs separately in the Isaac Teleop environment. The Placo/Pinocchio installation conflict
remains an environment consideration, not an unresolved Rung 3 integration choice.

This completion is for controller-to-simulator motion. It does not include robot camera
video in the headset, nor an unrecorded claim of exhaustive bilateral headset testing.

### Rung 4 — G1-29 sim → G1-23 sim
Per-embodiment variation is **data, not behaviour**: URDF, locked joints, EE parent + offset
(`wrist_roll` + 0.20 vs `wrist_yaw` + 0.05), rotation weight (0.5 vs 1.0), filter width
(10 vs 14). LeRobot has *one* copy of the IK — add the variant as a spec, not as copy #2.

Step 1-4 status: `assets/g1/g1_body23.urdf` is vendored from Unitree's LeRobot evaluation
assets, mesh lookup reuses the cached `lerobot/unitree-g1-mujoco` `assets/meshes` layout,
and `unitree_g1_lerobot/robots/g1_embodiments.py` contains a local G1-23 spec/IK implementation with the 10 active
arm joints from Unitree's `G1_23_ArmController`.

There are two side-by-side verifiers:

```bash
cd ~/lerobot-sim/unitree_g1_lerobot
./run_compare_g1_29_g1_23_urdf_mesh.sh
./run_compare_g1_29_g1_23_ik.sh
```

`run_compare_g1_29_g1_23_urdf_mesh.sh` is the stationary Step 3 check: G1-29 and native
G1-23 in neutral poses, with matching camera distance and angle for URDF/mesh inspection.

`run_compare_g1_29_g1_23_ik.sh` is the Step 4 verifier: left panel G1-29 IK on the existing
G1-29 MuJoCo scene, right panel G1-23 IK on a native G1-23 MuJoCo model compiled from the
vendored URDF. Its default motion profile cycles through arm up/down, forward/back,
left/right, and hand orientation changes, repeating continuously every 24 seconds.
The diagnostic targets use +/-12 cm X/Z and +/-10 cm Y offsets from each model's ready
pose; these are not full workspace limits. Frames are precomputed and replayed, so this
checks kinematics rather than physics or live control.

**2026-09-09 checkpoint:** the user has completed visual review of both verification
scripts. Native geometry and the scripted IK comparison have been reviewed. Preserve
these scripts as regression artifacts. This is not yet validation of a live G1-23
physics/DDS simulator: the comparison assigns joint positions and replays rendered poses.

#### What is actually left for Rung 4

Motor-config checkpoint: both variants now have source-derived motor profiles and a
supported-arm physics comparison suite. `run_compare_g1_29_motor_configs_no_gravity_compensation.sh` compares
the derived G1-29 gains against the active LeRobot defaults on identical models;
`run_compare_g1_29_g1_23_motor_configs_no_gravity_compensation.sh` compares both derived profiles on their native
models. These produce measured tracking/torque/oscillation reports and continuous
side-by-side replay. See [method and tests](motor-config-comparison.md).
This advances item 1 below as a standalone benchmark; it does not complete the live
LeRobot/DDS runtime or the transport/XR acceptance in items 2-4.

1. **Motor-driven simulation:** build the native G1-23 LeRobot/MuJoCo runtime with
   actuators, gravity, inertia, damping, and appropriate contacts. Verify pose holding
   and motion under motor commands; direct assignment to `qpos` is not this test.
2. **DDS and joint mapping:** exercise each of the ten active arm joints individually
   through a separate DDS command sender. Verify names, indices, directions, limits,
   unused transport slots, and measured joint feedback from the simulator.
3. **Scripted end-to-end motion, no headset:** route the existing Cartesian/orientation
   sweep targets through IK, DDS, and actuators. Compare target hand poses, commanded
   joint positions, measured joint positions, and hand poses computed from measurements.
   Separate IK residuals from actuator errors. Verify tracking, control timing, stability,
   and the intended response when commands stop or become stale.
4. **XR integration and regression:** select G1-23 in the existing bridge and verify
   headset control, startup diagnostics, engagement/release, and tracking loss/disconnect
   behavior. Verify supported arm control paths and retain working G1-29 behavior.
   Document the five-joint arm's position/orientation compromise and test results.

**No mandatory keyboard stage.** Keyboard Cartesian control reuses the same IK and merely
changes the source of targets. Scripted sweeps over DDS isolate the new physics,
actuator, transport, feedback, and timing behavior without introducing another input device.

**Next planning session:** decide the runtime integration details, controller settings,
verification artifact interfaces, and numerical pass/fail thresholds before implementation.
The items above are remaining acceptance work, not newly implemented capabilities.
Rung 4 control is complete only after live G1-23 simulation and XR acceptance, with
regression evidence and documented limitations. Visual feedback is tracked separately
as Rung 4V below; real robot work belongs to Rung 5.

Behind **golden-output tests**: the two implementations are not identical, and silently
normalising a weight changes robot behaviour.

Note a 5-DoF arm cannot reach an arbitrary SE(3) pose — 5 DoF against 6 objectives. The
solver returns a weighted projection and never reports unreachability, which is exactly what
the lower rotation weight encodes.

### Rung 4V - Robot Camera Feedback in the Headset

**Status: not implemented or verified.** Existing MuJoCo viewers render to Tk/X on the
workstation. The standalone simulator uses `publish_images=False`; the bridge's embedded
simulator also disables offscreen rendering and supplies no cameras. There is no local
camera-frame-to-XR-display pipeline. A connected client showing "Running" is not evidence
of robot video delivery.

Plan this after Rung 4 control acceptance and before moving to physical-robot work, so
rendering/streaming problems can be isolated from IK, DDS, and actuator problems. Preserve
the existing control path while adding a separate visual-feedback path:

1. Capture live frames from a defined robot camera in the simulator and verify them locally.
2. Choose the supported XR display/streaming integration and implement frame delivery to
   the headset. CloudXR controller connectivity alone is not a video implementation.
3. Verify in-headset that the image is the robot camera, updates during motion, has correct
   orientation/framing, and survives reconnects without blocking control.
4. Measure frame rate, frame age/latency, and the effect on simulation/control timing.

Mono versus stereo, fixed camera versus head-coupled view, transport details, and numerical
acceptance thresholds are decisions for the next planning session. No specific video API
or protocol is assumed by this plan. This milestone should serve both G1 variants and
remain separate from robot IK code for an eventual upstream contribution.

### Rung 5a - Physical G1-29 Baseline

Try a physical G1-29 first when one is already available, using the same embodiment as
the tested G1-29 simulation path. This isolates hardware transport, controller ownership,
mode transitions, gains/gravity compensation, command timing, and stop/disconnect behavior
before adding the G1-23 hardware differences. This is a planned test, not a hardware result.

[LeRobot's G1 documentation](https://huggingface.co/docs/lerobot/main/en/unitree_g1)
states that both 29- and 23-DoF variants are supported. The reason to start with G1-29
here is narrower: this checkout uses `G1_29_ArmIK` and G1-29 joint definitions in its main
robot path, and that is the XR/simulation path already exercised in this project.
Do not describe G1-23 as universally unsupported or assume published support validates
our custom XR bridge on hardware.

Plan hardware-specific operating modes, measured-pose initialization, arm-control weight
ramping, stale-command handling, and operator stop procedures against the actual robot
and SDK before actuation. Existing loopback-only simulation launchers are not physical
robot launchers. No physical commands are part of the structural refactor.

### Rung 5b - Physical G1-23

After G1-23 simulation acceptance and the G1-29 hardware baseline, apply the hardware
integration to G1-23. Reverify its joint mapping, limits, gains, end-effector definition,
and five-joint orientation compromise; G1-29 success does not validate these differences.
Keep the XR input and shared control path consistent with the simulation tests.

If G1-29 hardware is unavailable, record 5a as skipped and plan the common hardware
checks explicitly for 5b. Acquiring another robot is not a prerequisite for this project.

> If no G1-23 hardware is available, rung 4 is **sim-validated only**. Say so plainly in the
> PR rather than letting reviewers assume hardware validation happened.

---

## Lessons about the method itself

**A rung needing no devices is the cheapest and most valuable.** Rung 2 required no headset
and no robot, and surfaced seven blockers. Do device-free rungs first and in parallel.

**Skipping a rung is a real cost, paid later.** Rung 1 was skipped for a hardware reason;
rung 1' had to be invented to recover the isolation it provided.

**Split a rung when it spans two failure domains.** Rung 2 became 2a/2b because "IK is
broken" and "the simulator is broken" are different problems with different fixes.

**Suspect the test before the system.** Two of three pass/fail thresholds on rung 2a were
wrong, and both times the reported FAIL was a bad acceptance criterion, not a defect:
- First threshold measured residual on a *moving* target, so it was really measuring
  filter lag (2-step group delay).
- Second used a target translated while holding the original orientation — no guarantee
  such a pose is reachable, so the solver correctly returned a compromise.

On a validation ladder, a FAIL from a test you wrote five minutes ago is more likely to be a
bad test than a broken system. Derive targets from forward kinematics so a solution provably
exists, and separate *correctness* (static) from *lag* (moving).

**Native crashes need bisection, not reasoning.** `*** buffer overflow detected ***` with no
Python traceback was resolved by halving the space: plain DDS → DDS with minimal XML → DDS
with Unitree's XML. Three commands found a `<Tracing>` block.

**Instrument what you already know is suspect.** Rung 2a deliberately printed the
Pinocchio-vs-G1 joint order because a mismatch would have silently scrambled joints. It came
back `True` — one line that retired a whole class of hypothesis.

---

## Findings so far

| # | Where | Issue |
|---|---|---|
| 1 | LeRobot | `lerobot[kinematics]` conflicts with the conda Pinocchio the G1 docs require |
| 2 | LeRobot | docs reference `unitree_sdk2_python` on PyPI; no such distribution |
| 3 | LeRobot | cyclonedds C-library prerequisite undocumented |
| 4 | LeRobot | `G1_29_ArmIK` ships `reduced_robot.data` stale w.r.t. its own added frames |
| 5 | LeRobot | `WeightedMovingFilter` applies weights oldest-first — 2× intended group delay |
| 6 | LeRobot | IK failure path poisons the warm start and drops gravity compensation |
| 7 | LeRobot | `G1_29_ArmIK` requires `casadi` and `pinocchio.casadi`, but LeRobot does not declare or document that dependency |
| 8 | LeRobot Hub | `lerobot/unitree-g1-mujoco` has undeclared deps (`loguru`) |
| 9 | **Unitree** | `<Tracing>` in the default DDS config aborts on glibc 2.39 / Ubuntu 24.04 |

Items 5, 6 and 9 also exist in `xr_teleoperate` — 5 and 6 were copied verbatim into LeRobot.

**These were found by running the stack, not by reading it.** That is the argument for
offering hardware validation to maintainers: a G1 EDU with a non-Unitree tactile hand in a
working pipeline is a configuration almost none of them have, and it costs nothing to offer
since the hardware is running anyway.

---

## Immediate next actions

1. Plan the remaining Rung 4 physics/DDS work described above; visual review is complete.
2. Implement and verify the device-free simulator and scripted DDS path before headset testing.
3. Finish G1-23 XR acceptance and G1-29 regression checks.
4. Plan and validate Rung 4V camera feedback before proceeding to Rung 5 hardware work.
5. When available, establish a physical G1-29 baseline (5a) before physical G1-23 (5b).

Earlier rung notes retain historical bring-up observations; they are not the current
next-action list. Upstream issue reporting remains separate from Rung 4 acceptance.

## Companion documents

- [Rung 0 install notes](Rung0_isaac-teleop-install-notes.md) — rung 0 install
- [Rung 2 install notes](Rung2_lerobot-g1-mujoco-install-notes.md) — rung 2 install
- `unitree_g1_lerobot/diagnostics/rung2a_ik_only.py`, `unitree_g1_lerobot/diagnostics/rung2b_ik_to_mujoco.py` — validation scripts

## Production Teleop Design Note

Do not copy-paste `TeleVuerWrapper` into LeRobot as the production solution. Treat it as a
reference implementation for frame semantics: OpenXR basis to robot basis, optional
head-yaw anchoring, head/world to waist/IK-frame rebasing, bimanual wrist targets, and
validity fallback. The LeRobot implementation should be a small, tested retargeting layer
behind the existing `Teleoperator`/`Robot` APIs, so Isaac Teleop CloudXR, TeleVuer/Vuer, or
future XR inputs can feed the same wrist-target contract.

## Rung 3 Runtime Model

CloudXR is external for the current rung 3 test: `run_isaac_teleop.sh` starts the runtime,
and `python -m unitree_g1_lerobot.xr.rung3_xr_to_g1_mujoco --external-cloudxr` attaches to the existing OpenXR runtime.
MuJoCo is not external: the script constructs `UnitreeG1(UnitreeG1Config(is_simulation=True))`,
and `robot.connect()` creates the LeRobot G1 MuJoCo simulation in the same Python process
that runs XR polling, clutch retargeting, G1 IK, and `robot.send_action(...)`.
