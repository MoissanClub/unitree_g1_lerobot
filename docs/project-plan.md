# Execution Plan

## Goal and Current Baseline

Deliver G1-29/G1-23 VR teleoperation on LeRobot for simulation and physical robots,
with optional BrainCo, while minimizing long-term fork burden and reviewer burden.
[Goals](project-goals.md), [development model](development-model.md), and
[architecture](architecture.md) own those contracts; this file owns execution order.

Existing simulation mapping, IK, compensation, dual-arm XR and headset video are not
new bring-up tasks. Missing evidence includes Hub parity, systematic failure/performance
acceptance, physical operation, and hands. Historical manual video review did not
identify every embodiment/revision combination.

| Capability | G1-29 | G1-23 |
|---|---|---|
| Supported-arm native/DDS diagnostics | Existing numerical and visual evidence | Existing numerical and visual evidence |
| Native XR and camera | Implemented; historical user review | Implemented; historical user review is not exhaustive per-version evidence |
| Existing Hub path | Keyboard/DDS evidence; full Cartesian/XR parity open | Environment/model migration and full parity open |
| Physical VR | Authority selection and acceptance pending | Backend validation and separate acceptance pending |
| Optional BrainCo | SDK contracts exist; physical acceptance pending | SDK contracts do not establish embodiment-specific physical acceptance |
| Articulated BrainCo simulation | Not implemented | Not implemented |

The frozen fork checkpoint `d89a1d0b` passed 407 targeted tests plus a separate GPU
test, not a new headset/hardware review. The operator release remains `d5e400bc`.
[Verification evidence](branch-stack-verification.md) records exact scope and commits.

## Product Milestones

| Milestone | Outcome | Exit evidence |
|---|---|---|
| P0: Scope and ownership | Finalize baseline/reference match-defer matrix; assign owners and ledger rows | Explicit supported/gated/deferred combinations and next gates |
| P1: Preserve delivery | Keep both embodiments' current pinned simulation/XR/video usable | Installation, headless control/camera, manual release review |
| P2: Upstream-aligned simulation | Agree environment placement and implement G1-23 parity; verify G1-29 Hub Cartesian/XR | Both arms, mappings, compensation, cameras and failure behavior; headless plus manual evidence before replacing native workflow |
| P3: Physical delivery | Select explicit authority early; staged hardware bring-up per embodiment | Read-only preflight, bounded joint control, scripted IK, XR and camera acceptance under declared limits |
| P4: Hands and recording | Optional BrainCo hardware and standard LeRobot recording | Per-side mapping/lifecycle, chosen dataset schema, replay/readback; explicit simulation-hand limitations |
| P5: Extensions | Mobile manipulation, extra tracking modes, SONIC where supported | Separate controller/embodiment evidence; no implication of baseline acceptance |

P2 and P3 design run in parallel with upstream submissions. Product video delivery
does not wait for hand API polish or upstream review. Physical G1-29 is the preferred
baseline before G1-23 when hardware is available; it is not inherited G1-23 acceptance.
Record unavailable hardware rather than pretending a stage passed.

## Next Actions

1. Create the delivery ledger in the LeRobot integration branch: owners, commits,
   dependency/PR destination, exact evidence and remaining fork delta.
2. Continue #4664 on `g1/bugfixes`; prepare `submit/keyboard-arm-control` with its
   bugfix prerequisite explicit. Recheck live PR state before edits.
3. Clean #4651's actual embodiment diff and dependency description. The dq merge
   alone does not remove inherited keyboard changes; wait for that prerequisite
   or extract embodiment-only changes deliberately.
4. Start G1-23 simulator placement and physical authority design now. Preserve
   working native functionality until parity; do not begin hardware motion from this plan.
5. Resolve standard-loop XR feedback compatibility and verify processor wiring
   against the Hub path before requesting the XR review.
6. Continue the [non-physical acceptance backlog](future-non-physical-work.md)
   without conflating it with PR acceptance or redoing proven bring-up.

## Small Upstream Contributions

Preferred review order is not a compulsory linear branch stack. Base each submission
on accepted upstream prerequisites, or explicitly declare a temporary submission parent.

| Contribution / branch | Prerequisite and reviewer focus | Required verification |
|---|---|---|
| dq fix / existing `g1/bugfixes` | Existing motor path; minimal field correction | Real SDK serialization regression |
| Keyboard / `submit/keyboard-arm-control` | Existing G1-29 Hub simulator and command correctness | Standard CLI/terminal input, independent arms, hold/disable; no new solver/backend |
| Embodiments / existing `g1/embodiments` | Existing robot/config; compatible defaults and sparse mapping | G1-29 regressions, G1-23 mapping/guards, runnable backend evidence when enabled |
| Cartesian / future `submit/cartesian-processor` | Accepted embodiment capability, not native simulator | Existing IK adaptation, frames/limits/measured state/failure handling; actual Hub control |
| XR / future `submit/xr-dual-arm` | Cartesian capability and shared device/session lifecycle | Real standard loop, independent clutches, tracking loss, Hub physics, no-video entry point |
| Hands / future `submit/hand-composition` | Robot composition, not XR/IK/locomotion selection | Feature dispatch, lifecycle, failure contracts; one agreed API |
| BrainCo / future `submit/brainco-hands` | Accepted hand contract; optional SDK | Units/mapping and SDK errors, then separate physical evidence |
| Video / future `submit/xr-video-adapter` | Agreed camera/session ownership; not hands | Frame age/recovery, pixel checks, physical network path and manual headset acceptance |

Keep environment/model changes in the agreed simulator repository and session/graphics
changes with Isaac Teleop. A complete product demonstration may compose these small
PRs; do not bundle three new public contracts into one review.

## Decision and Verification Gates

- Before publishing Cartesian APIs: settle solver reuse and frames/tool transforms;
  use already-tested G1-23 requirements without speculative abstraction.
- At physical design kickoff: establish authority and watchdog/handover semantics.
  This does not block independent simulation or hand-composition PRs.
- Before hand API submission: agree per-side configuration, names/units, lifecycle,
  body-controller feature interaction and fault behavior.
- Before video API submission: agree session/graphics ownership; preserve working delivery.

Each PR needs an exact base SHA, dependency/model pins, working directory, executable
commands, expected outcomes and resulting evidence in the ledger and PR description.
The [historical command inventory](branch-stack-commands.md) is a starting point,
not proof for newly extracted Hub/physical paths. Add missing commands as those
implementations exist; do not invent runnable examples for unimplemented gates.

Keep automated headless, manual viewer/headset, hardware, and installation evidence
separate. Never relabel native physics results as Hub acceptance, mocked hands as
grasp physics, or a clean process exit as a verified physical stop.
