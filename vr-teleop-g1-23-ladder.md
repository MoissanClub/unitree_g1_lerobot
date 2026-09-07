# VR Teleoperation of the Unitree G1-23 on LeRobot — Validation Ladder

Working method and status log. September 2026.

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
| **G1-23** | The base G1 is what schools and individual builders can afford, and it is unsupported everywhere. The 29-DoF EDU is not the accessibility problem. |
| **Not forking `xr_teleoperate`** | Vendor sample code — no tests, no CI, 200 lines duplicated seven times. Good for bug fixes, hostile to architecture. |

### What already exists vs. what must be built

```
Isaac Teleop XR  →  LeRobot  →  SO-101          ✅ ships today
Isaac Teleop XR  →  Isaac ROS/Jetson  →  G1     ✅ ships today (NVIDIA's own stack, not LeRobot)
Isaac Teleop XR  →  LeRobot  →  G1              ❌ the gap — this project
```

Isaac Teleop is a **pure input device**: it streams controller poses over CloudXR and never
touches a simulator. Whether the far end is MuJoCo or real hardware is invisible to it.
`XRController.get_action()` returns `{grip_pos, grip_quat, squeeze, trigger}` already
rebased into the robot frame. The interface between input and retargeting is a **4×4
homogeneous wrist pose** — the same seam `xr_teleoperate` uses with a completely different
input device.

---

## The ladder

| # | Change | Proves | Devices | Status |
|---|---|---|---|---|
| **0** | CloudXR + headset, NVIDIA's own example | The headset works at all | headset | ⚠️ partial |
| **1** | Shipped SO-101 example | Isaac Teleop → LeRobot path | + SO-101 | ⏭️ skipped |
| **1'** | Same headset via LeRobot's `XRController` | The LeRobot binding, minus actuators | headset | ⬜ todo |
| **2a** | Script → `G1_29_ArmIK` | IK stack, no sim, no XR | **none** | ✅ done |
| **2b** | 2a → MuJoCo | Robot interface + sim | **none** | ✅ done |
| **3** | Join 1' + 2b | **The glue — the actual contribution** | headset | 🟡 in progress |
| **4** | sim → real G1-29 | Transport, FSM, arm_sdk, safety | + G1-29 | ⬜ todo |
| **5** | 29 → 23 | The spec-as-data refactor | + G1-23 | ⬜ todo |
| **6** | Gripper → BrainCo hand | Tactile integration | + BrainCo | ⬜ todo |

Rungs 1' and 2 are independent — one needs the headset, the other needs nothing — so they
can run in parallel or in either order.

---

## Rung detail

### Rung 0 — CloudXR + headset, no robot
Run NVIDIA's `gripper_retargeting_example_simple.py`. See `isaac-teleop-install-notes.md`.

**Acceptance:** session stable 5+ min · both controllers tracked · squeeze/trigger sweep
0→1 · poses continuous · tracking survives torso rotation and full arm extension.

**Status:** CloudXR runtime 6.3.0 and WSS proxy running; EULA accepted.
**Headset connection not yet verified.** Quest Pro is named in neither doc set — the risk
that gates everything downstream. Fallback: Quest 3 or PICO 4.

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
`lerobot-g1-mujoco-install-notes.md`. **Requires a real X session or `xvfb-run`.**

### Rung 3 — the contribution
`grip_pos`/`grip_quat` → 4×4 → `solve_ik` → 29-joint action dict → `UnitreeG1(is_simulation=True)`.

Both halves proven, so any failure is in ~20 lines of glue.

Open design question: route through LeRobot's generic **Placo** Cartesian pipeline (idiomatic,
reuses existing plumbing) or through **`G1_29_ArmIK`** (bimanual, already G1-tuned)? For two
arms the CasADi one likely wins — but lift `clutch.py` from the SO-101 example either way.
Squeeze-to-engage is the "don't fight the robot while repositioning" mechanism you would
otherwise reinvent.

⚠️ `lerobot[kinematics]` (Placo) conflicts with the conda Pinocchio the G1 IK needs. **Rungs
1' and 2 currently require two separate environments.** Rung 3 is where that has to be
resolved.

### Rung 4 — sim → real G1-29
Flip `is_simulation=False`, set `robot_ip`, run `run_g1_server.py`. Everything above the
transport is unchanged. This is where FSM state, `arm_sdk` weight ramping, and the
protective layer become real. Prerequisites: robot in **FSM 500** (Main Operation Control,
`L2+B` → `L2+UP` → `R1+X`); seed `motor_cmd` from measured `q` before setting the weight;
a stale-command watchdog.

### Rung 5 — 29 → 23
Per-embodiment variation is **data, not behaviour**: URDF, locked joints, EE parent + offset
(`wrist_roll` + 0.20 vs `wrist_yaw` + 0.05), rotation weight (0.5 vs 1.0), filter width
(10 vs 14). LeRobot has *one* copy of the IK — add the variant as a spec, not as copy #2.

Behind **golden-output tests**: the two implementations are not identical, and silently
normalising a weight changes robot behaviour.

Note a 5-DoF arm cannot reach an arbitrary SE(3) pose — 5 DoF against 6 objectives. The
solver returns a weighted projection and never reports unreachability, which is exactly what
the lower rotation weight encodes.

> If no G1-23 hardware is available, rungs 5–6 are **sim-validated only**. Say so plainly in
> the PR rather than letting reviewers assume otherwise.

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

1. Finish rung 0 — connect the headset, run the acceptance checklist, settle the Quest Pro question
2. Rung 1' — ten lines, no purchase
3. File findings 1–3, 7 and 8; file 9 with Unitree
4. Rung 3 — the glue

## Companion documents

- `isaac-teleop-install-notes.md` — rung 0 install
- `lerobot-g1-mujoco-install-notes.md` — rung 2 install
- `rung2a_ik_only.py`, `rung2b_ik_to_mujoco.py` — validation scripts

## Production Teleop Design Note

Do not copy-paste `TeleVuerWrapper` into LeRobot as the production solution. Treat it as a
reference implementation for frame semantics: OpenXR basis to robot basis, optional
head-yaw anchoring, head/world to waist/IK-frame rebasing, bimanual wrist targets, and
validity fallback. The LeRobot implementation should be a small, tested retargeting layer
behind the existing `Teleoperator`/`Robot` APIs, so Isaac Teleop CloudXR, TeleVuer/Vuer, or
future XR inputs can feed the same wrist-target contract.
