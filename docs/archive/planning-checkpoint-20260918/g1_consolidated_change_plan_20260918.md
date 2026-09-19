# G1 / LeRobot consolidated change plan

> Historical snapshot, archived during the documentation consolidation. Not current
> setup, branch, or execution instructions. See the [current documentation](../../README.md).

Discussion checkpoint: September 18, 2026.

## Scope and objectives

This document consolidates the discussion and branch audit; it is not a new audit of moving branch heads or a record of changes already executed. The inspected LeRobot upstream/fork baseline was `5aa74557f84c54d4b458f8b9643c5aa2982acfed`. The tested keyboard-first integration checkpoint was `d89a1d0b5f628045cc73293bcdfa2dc6c0808549`.

Product goal: deliver VR teleoperation of **G1-29 and G1-23 on the LeRobot stack, in simulation and on physical robots, with and without BrainCo hands**, comparable in explicitly selected capabilities to [Unitree xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate). Better architecture means reusable device/processor/robot boundaries, standard teleoperation and recording workflows, explicit control ownership, and reproducible verification, not a second independent teleoperation framework.

Optimize delivery against two criteria: maximize work acceptable to the owning upstream projects while minimizing long-term fork maintenance; and minimize reviewer burden through familiar implementation patterns, bounded changes, and independently reproducible evidence. Upstream acceptance supports the product goal; it is not a substitute for delivering it.

Objectives:

1. Maximize contributions accepted by LeRobot or the dependency that owns the relevant functionality.
2. Minimize permanent fork-only implementation, especially duplicated runtimes and public APIs.
3. Minimize rework of an interface immediately after the PR that introduces it.
4. Preserve the working system, tests, release pins, and collaborators' work throughout migration.
5. Deliver both embodiments without depending on an uncertain SONIC or upstream review timeline. Track every necessary downstream delta and its replacement or continuing-maintenance rationale.

Status terminology:

- **Confirmed in inspected code:** directly supported by the source examined during the discussion.
- **Working design choice:** recommended for this project; not necessarily an upstream-approved API.
- **Open:** needs implementation validation and/or maintainer agreement before dependent public APIs are finalized.

### Product milestones, separate from PR order

The proposed initial baseline is stationary or support-backed bimanual controller-based VR, with camera feedback and optional hands. This is a scope proposal, not a claim that mobile manipulation is unnecessary or already covered. Define match/defer decisions for Unitree's controller input, hand tracking, camera display, recording, hand devices, and motion mode before claiming comparable functionality.

| Milestone | Deliverable and exit evidence |
|---|---|
| P0: Scope and evidence | Maintain the capability matrix below, identify a responsible owner for each gap, and record baseline versus later features. |
| P1: Preserve working delivery | Keep both embodiments' native arm simulation, dual-arm XR, and headset video usable on a pinned integration release. Record automated and manual evidence separately. |
| P2: Upstream-aligned simulation | Coordinate G1-23 model/environment support with the simulator owner; validate both embodiments through public LeRobot APIs, including camera and control behavior. |
| P3: Physical delivery | Select explicit command authority early; validate transport, timing, disconnect behavior, compensation, and network camera delivery for each embodiment before enabling its supported hardware configuration. |
| P4: Hands and recording | Validate optional BrainCo hardware independently and verify the selected standard LeRobot recording workflow. Mock hand transport is not articulated-hand physics evidence. |
| P5: Extensions | Track mobile manipulation, skeletal hand input, and SONIC integration separately, with explicit embodiment and controller compatibility. |

P2 and the design work for P3 proceed in parallel with submissions. Product camera delivery does not wait for hand API polish or for an upstream video PR to merge. One product milestone may compose several small PRs; do not bundle unrelated contracts merely to make a PR end-to-end.

### Capability matrix and current evidence

| Embodiment | Native arm simulation, no BrainCo | Hub-path parity | Physical VR, no BrainCo | BrainCo simulation / physical |
|---|---|---|---|---|
| G1-29 | Existing mapping, IK, compensation, dual-arm XR; user-reviewed headset video | Keyboard Hub/DDS evidence exists; full Cartesian/XR/video parity still needs verification | Not established by the simulation evidence | Simulation physics not established; physical acceptance pending |
| G1-23 | Existing mapping, IK, compensation, dual-arm XR; user-reviewed headset video | Model/runtime migration and full workflow verification required | Not established by the simulation evidence | Simulation physics not established; physical acceptance pending |

These are project-history claims, not fresh reruns or proof for every branch. Attach exact code/model revisions, commands, and reports to each accepted cell. Track hand transport, articulated physics, and physical grasp behavior as different evidence classes. The missing G1-23 work is Hub and physical parity, not a first implementation of its IK.

## 1. Development-process changes

### 1.1 One authoritative integration branch, not literally one Git branch

Use `MoissanClub/lerobot` as the source of reusable LeRobot implementation. Maintain one active working integration branch plus short-lived task and submission branches.

| Branch/category | Role | Rule |
|---|---|---|
| `main` | Clean upstream mirror | Update by fast-forward; do not develop here or force-reset it as routine maintenance. |
| `dev/g1-integration` | Authoritative combined working system | Created from `d89a1d0b5f628045cc73293bcdfa2dc6c0808549` (tested keyboard-first checkpoint); keep integration history stable. |
| `work/<feature>` | Short-lived collaborative development | Review and test changes before integrating them. |
| `submit/<feature>` | Curated upstream contribution | Base on the appropriate accepted upstream state; no unrelated experimental prerequisites. |
| Existing archives / acceptance checkpoints / release pins | Reproducibility and migration safety | Preserve; do not treat them as additional products requiring parallel feature development. |

Do not assume the existing generic `integration/g1-acceptance` ref is the newest checkpoint. Record the selected SHA explicitly.

Keep `MoissanClub/unitree_g1_lerobot` for operator setup, launch configuration, acceptance tools, experiments, and temporarily retained compatibility code. Gradually remove duplicated reusable implementation only after its replacement is validated. Do not maintain two independently evolving production copies of the same G1 feature across the two repositories.

### Branch inventory after initial cleanup

The following ref-only changes were executed after this plan revision. No feature was reimplemented or removed, and no new regression result is claimed.

| Branch | Current role and state |
|---|---|
| `main` | Existing upstream mirror; unchanged. |
| `dev/g1-integration` | Created and published at `d89a1d0b5f628045cc73293bcdfa2dc6c0808549`; active working branch in `lerobot-upstream`. |
| `g1/bugfixes` | Retained as the existing submission branch for open upstream PR #4664; do not create a duplicate `submit/dq-bugfix` just for naming consistency. |
| `g1/embodiments` | Retained for open upstream PR #4651. Still needs the scope/dependency cleanup described below; do not break the PR by deleting its head. |
| `submit/keyboard-arm-control` | Created and published at `2343d572a18f05fd658d4cae19838c4c096d1194`; contains the already-tested keyboard feature on the bugfix base, with that prerequisite still explicit. |
| `submit/cartesian-processor`, `submit/xr-dual-arm` | Future curated submissions on their actual prerequisites, not created as aliases of the old simulation-dependent branches. |
| `submit/hand-composition`, `submit/brainco-hands`, `submit/xr-video-adapter` | Create on demand after their respective contracts and extraction scopes are ready. |
| `work/g1-23-sim-parity`, `work/g1-physical-authority` | Proposed short-lived implementation branches from integration when work starts; simulator-owned changes belong in the simulator repository. No empty placeholder branches created. |

Open PR heads were checked through GitHub's API during cleanup; this is a dated observation, not a permanent claim about their state. Existing PR branch names are exceptions to the preferred `submit/*` convention.

Deleted 14 local scratch branches: eight under `keyboard-stack/20260918/*` and six under `reorder/20260918/*`. Their exact tips were first saved and published as `retired/20260918/<old-branch-name>` tags, then verified before deletion. These scratch branch names did not exist on origin, so no remote branch deletion was needed.

Retained the remaining old `g1/*` feature refs as frozen extraction/verification inputs: `tools/verify_lerobot_branch_stack.py` still consumes their names. Retained `archive/*` refs, all `integration/g1-acceptance*` checkpoints, installer pins, and worktree-backed branches. The development workspace also references `g1/simulation`, `g1/bugfixes`, and the older acceptance checkpoint. Removing these now would break existing workflows. Retire them separately only after consumers migrate and accepted replacements are recorded; they are not parallel active development lines. The user's `~/lerobot-dev/lerobot` checkout was not switched or edited.

### 1.2 Normal change flow

1. Develop a coherent change in a short-lived branch and test it in the working integration system. An independently upstream-ready fix may start in its submission branch instead, then be integrated into the working system.
2. Keep commits separated by concern. Do not combine an IK change, simulator behavior change, and XR/video refactor in one commit.
3. For upstream delivery, create a clean `submit/*` branch and cherry-pick feature-only commits where they are self-contained. Otherwise port the relevant hunks deliberately. Do not cherry-pick an integration merge or copy an entire final file that includes unapproved sibling work.
4. Limit concurrent review according to maintainer bandwidth, normally one active prerequisite per dependency chain. Independent contributions may proceed in parallel. This is workload policy, not a technical dependency. Record reviewer-driven changes and reconcile them back into the working integration implementation promptly; do not leave two competing versions.
5. After a merge, synchronize `main`, reconcile the integration branch with accepted upstream changes, and prepare the next PR against actual upstream. Retire the equivalent fork-only patch rather than deleting its functionality. Preserve the old parent boundary when transplanting a child after a squash merge.

Cherry-picking is a delivery mechanism, not a reason to maintain permanent duplicate implementations. Maintain a delivery ledger at `docs/delivery-ledger.md` in `MoissanClub/lerobot`: feature, responsible owner, source and accepted commit(s), dependencies, target repository/PR, acceptance evidence, remaining fork-only delta, and replacement condition or justified long-term ownership. Publish pinned downstream releases when ready rather than waiting for all upstream merges. Necessary plugins and operator tools are acceptable; independently evolving copies of core runtimes are the maintenance risk.

After an upstream squash merge, record the old parent and accepted SHA, merge upstream into the shared integration history, and reconcile changes semantically. Do not blindly revert the original feature or take entire upstream files: sibling downstream features may share those files. Audit the remaining diff against the ledger and rerun integration. Use `rebase --onto` only for clean linear submission children; curate explicit patches for mixed histories. Keep shared integration history stable.

### 1.3 Stop rebuilding the complete published branch stack

The existing `g1/*` branches remain extraction/reference material during migration. Do not rewrite all nine branches whenever a PR order changes. Freeze old refs, create submission branches, and archive obsolete development lines only after confirming that no collaborator or release depends on them. Retire a `g1/*` branch once its content is represented in an accepted upstream PR and the delivery ledger records the accepted SHA; do not carry it forward indefinitely as a parallel product.

Review order and software dependencies are different. Hands do not need to depend on XR merely because the hand PR is reviewed later. A simulator test fixture does not need to become a production dependency of every feature it tests.

### 1.4 Sequential-PR validation

Each next PR should add one capability relative to upstream containing the preceding accepted merge. Check the actual diff, not only a branch name or original commit message. Test the squash-and-transplant case as well as merge-commit integration.

Preserve the source-parent SHA before any transplant. When history is mixed, curate an explicit patch series rather than applying a broad rebase. Inspect rewritten patch series and coordinate published-head updates; never routinely rewrite the shared integration branch.

The existing draft embodiment PR's dependency/scope description must match its actual diff. At the audited snapshot, #4651 inherited keyboard work even though its description excluded teleoperation. The dq merge alone cannot remove keyboard changes from that diff. Keep the inherited dependency explicit until keyboard is accepted, or deliberately extract embodiment-only changes onto the intended base. Verify the resulting embodiment PR diff before requesting review; do not infer scope from its title. PR status must be rechecked before changing it.

## 2. Simulation: standardize the production path, retain the diagnostic harness

### Confirmed in inspected code

The existing G1 Robot-facing simulation path already loads a Hub-distributed environment:

```text
UnitreeG1(is_simulation=True)
    -> make_env(config.sim_env, trust_remote_code=True)
    -> lerobot/unitree-g1-mujoco code/assets
    -> local MuJoCo execution
```

"Hub" is distribution, not cloud execution. Body commands and measurements use the simulator's DDS bridge. In the inspected LeRobot path, the background subscription loop advances the environment. The Robot-facing G1 path and the Hub environment are layers of the same implementation, not evidence of two wholly separate G1 simulators.

### Working design choice

Use this upstream Hub/DDS path for upstream-facing G1-29 keyboard, Cartesian, and XR acceptance. Do not redesign generic `Robot`, `Env`, or physics-clock ownership for the first contributions.

Keep the native `g1_simulation.py` implementation as the working downstream arm-simulation path during migration and as diagnostic/test infrastructure where still needed afterward. It fixes legs, waist and fingers, removes collision geometry, rejects locomotion, and advances time with different semantics. It is useful for deterministic arm verification, not evidence of whole-body or articulated-hand simulation parity.

Remove native simulation from the required production ancestry of Cartesian, XR and hand support. Base Cartesian submissions on accepted embodiment capabilities, and XR submissions on accepted Cartesian capabilities, not directly on frozen `g1/*` refs. If deliberately stacked before acceptance, declare the corresponding `submit/*` parent and later transplant only the child changes. The existing feature branches are extraction material, not clean submission bases. Keyboard is independently useful, not an inherent IK/XR dependency.

Create an owned G1-23 simulator-parity workstream in the delivery ledger. First agree placement with the existing Hub environment maintainers rather than assuming ownership of another permanent backend. Track model/assets, sparse DDS mapping, environment selection, supported fixed-base behavior, dependency pins, and test evidence. Keep the working downstream production path until both-arm joint/IK behavior, compensation, camera delivery, headless operation, and manual viewer/headset workflows are validated on its replacement. A load-and-step smoke test is necessary but insufficient. Retain native diagnostic capabilities that have a named verification purpose; do not require identical whole-body dynamics from a fixed-base harness.

## 3. Locomotion: do not silently select a different controller for real hardware

### Confirmed in inspected code

The documented upstream G1 workflow can use LeRobot-side learned body controllers in both simulation and hardware. The code examined includes GR00T/Holosoma and a separate SONIC whole-body mode.

Therefore this is **not** an established upstream rule:

```text
simulation -> GR00T
real robot -> Unitree stock high-level locomotion
```

It was an alternative proposed in the discussion.

### Working design choice

The first XR contribution is G1-29 arm control in the existing Hub simulator with torso support retained. Do not make walking a prerequisite of that PR.

For later decoupled locomotion, prefer the existing upstream controller route as the upstream-aligned starting point:

```text
XR joystick / another action source
    -> existing locomotion action fields
    -> compatible LeRobot RobotController
    -> body joint targets
    -> Hub/DDS simulator OR validated physical transport
```

Keep device-specific joystick processing outside the body driver. Do not put simulation-versus-hardware routing in the XR reader.

### Open decisions

- Whether this project's first physical deployment uses LeRobot's learned controller or Unitree's stock locomotion service with a compatible arm-only command path.
- How any stock-control backend is exposed and validated. Treat it as an explicit control mode, not an undocumented consequence of `is_simulation=False`.
- Control authority, onboard timing, stale-input behavior, controller compatibility, and physical acceptance requirements.

Assign the physical-control design work at P0/P3, not only when the hand PR is ready. Identify which process owns arms, waist, legs, and hands in each supported mode, and which watchdog remains effective if workstation/XR input stops. Treat supported robot-side locomotion with an arm-only path as an explicit candidate alongside existing LeRobot controllers; verify compatibility for each embodiment rather than silently selecting a backend. This design work runs early, while actual hardware enablement remains separately gated in section 10.

The arm command interface and the locomotion interface are separate responsibilities. A common high-level command schema does not establish equivalent dynamics when different controllers are used in sim and hardware.

G1-23 embodiment metadata does not establish GR00T, Holosoma, SONIC, or physical-runtime compatibility. Keep unsupported combinations explicitly gated.

## 4. Cartesian arms: IK in the processor, solver reusable, execution downstream

### Confirmed and retained as a working design

The Cartesian branch already includes `G1CartesianActionProcessor`, registered as `g1_cartesian_to_joints`. It uses measured arm joints and target poses to produce named joint commands.

Use the same representation/kinematics boundary in simulation and hardware:

```text
XR pose input
    -> frame mapping + clutch/rebasing
    -> Cartesian EE targets
    -> G1 robot action processor
    -> reusable IK solver
    -> named arm joint targets
    -> UnitreeG1 execution
    -> upstream simulator OR appropriate physical command path
```

The processor may use measured feedback and retain solver state; "processor" does not mean "stateless" or "no feedback." Its responsibility is the action adaptation boundary, not ownership of the whole-body servo loop.

Direct joint-space teleoperators/policies need not pass through IK. SONIC body mode is an alternative, not a consumer of this independent arm-target path.

Gravity feedforward belongs to execution/model handling downstream of XR. Avoid competing implementations applying it twice. Shared helper code may be used by execution and diagnostic tools.

### Decisions to settle before dependent XR APIs

- How `G1ArmKinematics` and upstream `G1_29_ArmIK` share implementation, preserve backward compatibility, or expose an explicitly selected solver.
- Pose frame, units, quaternion convention at serialization boundaries, tool/EE transform, and measured-state seeding.
- Failure reporting, joint limits, and whether a bound is per-call displacement or time-based velocity.
- Moving-waist/base handling. The current solver locks non-arm joints at neutral; do not assume that it already supports coordinated walking and reaching correctly.

Prefer adapting the existing upstream IK implementation and helper interfaces before introducing a second public solver framework. Use the already-tested G1-23 requirements to check the design before publishing it. Settle frames, units, tool transforms, measured-state input, and joint naming early so the second embodiment is additive. Introduce a solver protocol only when actual implementations require it, not as speculative SONIC preparation.

Do not strip out the embodiment selector simply to reintroduce it in the next PR. Keep embodiments before Cartesian/XR. The audit found little benefit in reversing the already-tested keyboard/embodiment order.

Keep the first delivered robot-facing action space compatible with existing joint actions. Current intermediate 4x4 EE targets are not, by themselves, a complete Cartesian ACT training/recording schema. Treat that extension separately.

## 5. Hands: end-effector configuration is established; physical I/O composition is still a proposal

### Confirmed in inspected upstream code

G1 has an `end_effector` selector identifying `dummy`, `dex1`, or `dex3`. The reviewed commit uses it to select simulation models, finger actuators and associated cameras; it does not establish a general physical `EndEffector` API.

### Working design choice

Expose one top-level `UnitreeG1` to teleoperation, recording and policy rollout. Compose modular hand devices underneath it, merge observations/features, and dispatch hand actions separately from body commands.

The current BrainCo implementation is per-side:

```text
UnitreeG1 : Robot
    body / arms
    hands
        left  -> BrainCoHand
        right -> BrainCoHand
```

This preserves independent hand implementation/testing without asking generic rollout to orchestrate two unrelated top-level robots. Whether a reusable hand also implements LeRobot's `Robot` interface is secondary and not settled by the end-effector enum.

Keep hand input adaptation separate from transport:

```text
trigger/button OR tracked fingers
    -> closure mapping / retargeting
    -> hand-native motor targets
    -> selected hand driver
```

Hands must remain independent of XR, Cartesian IK and the native arm simulator. They should compose with raw-joint, locomotion-plus-arms, and SONIC-token body modes without assuming one body action schema.

### Open decisions

Agree with maintainers on per-side versus bilateral configuration, selector/driver consistency, names and units, lifecycle, failure handling, and timing. `HandSystem`, `HandCollection`, or a future `EndEffector` protocol are proposals, not names guaranteed by upstream.

Introduce only one composition API upstream. Omit the 153-line compatibility-only `G1WithHands` wrapper from the upstream proposal; retain it downstream only for released consumers that need it. Finalize the base configuration contract before the BrainCo implementation PR so that the child need not immediately redesign it.

Physical hand I/O and articulated-hand simulation are separate implementations. The audited code rejects BrainCo simulation and fixes fingers in the native simulator. Do not claim hand-contact simulation or physical grasp safety based on model selection or fake-driver tests. Review vendor SDK blocking and whole-body failure behavior separately; an outer hand call does not necessarily block an independent body-controller thread.

## 6. SONIC: preserve compatibility without making it a dependency

In the LeRobot code examined, SONIC is a token-decoder whole-body mode: a 64-dimensional motion token plus robot-state history produces commands for all 29 body joints. Hand motors are separate.

```text
Decoupled mode:
    locomotion controller -> lower-body/waist targets
    Cartesian IK          -> arm targets
    hand adapter          -> hand motors

SONIC mode:
    motion token -> SONIC -> legs + waist + arms
    hand adapter          -> hand motors
```

Do not run independent IK and SONIC as competing writers to the same arms. SONIC is not evidence that upstream has deprecated the decoupled mode. XR-to-SONIC encoding/reference generation is a separate integration, not included by selecting the current decoder controller.

The product does not wait for SONIC VR or G1-23 SONIC support. Decoupled arm IK is a supported delivery path, not disposable scaffolding. Share device acquisition, camera handling, and optional hands where actual contracts allow; do not force joint targets and latent tokens into a new common action abstraction. A perceived SONIC direction is not a confirmed LeRobot roadmap or a commitment to support G1-23.

## 7. XR and video: reuse device/runtime infrastructure and make each PR independently usable

The audited XR reader already exposes both controller poses, squeeze, trigger, tracking and time. It does not yet expose joystick locomotion axes or skeletal finger tracking.

For the first XR PR:

- Reuse or extend the shared upstream Isaac Teleop session/device lifecycle where practical.
- **Acceptance prerequisite:** Resolve the standard G1 loop's feedback call versus the XR reader's rejection of nonempty feedback before requesting review of the XR submission. This is a functional incompatibility, not a style issue; develop and verify the fix as part of the scoped work.
- Provide tested processor construction and entry-point registration.
- Move a working no-video example into the XR PR; do not wait for the video child.
- Use public robot observations/actions against the Hub simulator, not `robot._native` or private MuJoCo arrays.

For video, decide which code belongs in LeRobot versus NVIDIA Isaac Teleop. Shared OpenXR input/render session management is a platform concern; LeRobot should ideally keep a small camera adapter. The current mmap RGB channel is same-host transport, not a robot-to-workstation network stream.

Do not replace a working video path before validating its replacement, but avoid making an additional frame protocol and duplicated SDK lifecycle permanent core dependencies without agreement.

## 8. Proposed upstream submissions and ownership

This is a preferred review sequence, not a mandatory linear dependency chain or a promise of maintainer acceptance. Keep each PR to one bounded behavior change using familiar upstream patterns. Include a runnable example or focused reproducer, default/backward-compatibility checks, optional-dependency behavior, and an explicit list of unverified combinations. Do not introduce a simulator, solver rewrite, or generic composition framework in a PR whose purpose is an input adapter.

| Order | Contribution / proposed branch | Actual prerequisites and reviewer goal | Evidence required on its intended base |
|---:|---|---|---|
| 1 | SDK `dq` bugfix / existing `g1/bugfixes` | Existing upstream motor command path; smallest correct field fix. `g1/bugfixes` at `e272f385` contains `f257939` and `e272f38` and backs #4664; preserve that PR head. | Serialized SDK command regression with real message types. |
| 2 | Keyboard / `submit/keyboard-arm-control` | Existing G1-29 Hub path and command correctness; no new solver/backend. Useful independently of XR. | Standard CLI, real terminal input, measured movement of both arms, hold/disable and regression tests. |
| 3 | Embodiments / existing `g1/embodiments` | Existing robot implementation; explicit mappings and compatible defaults. Disclose/remove inherited keyboard ancestry as appropriate; verify #4651 scope. Coordinate G1-23 simulator support without claiming unsupported controllers. | G1-29 regression, G1-23 sparse mapping and rejected invalid configurations; linked runnable simulator evidence when support is enabled. |
| 4 | Cartesian / `submit/cartesian-processor` | Accepted embodiment capability, not native simulation. Reuse IK and settle maintainer question 1 before finalizing the public contract. | Frame/units, bounds, measured-state seeding and failure tests; actual G1-29 Hub control plus G1-23 diagnostic regression. |
| 5 | XR / `submit/xr-dual-arm` | Cartesian capability and shared Isaac lifecycle; resolve feedback compatibility. Keep device extension and G1 wiring separately scoped if they have different owners. | Both independent clutches through the standard loop, processor construction/registration, Hub physics, tracking-loss behavior, and a working no-video example. |
| 6 | Hands / `submit/hand-composition` | Robot composition only; no XR, IK, or simulator dependency. Settle maintainer question 3; avoid compatibility-only wrapper. | Feature/action dispatch, lifecycle and fault contracts; distinguish fake-device tests from hardware claims. |
| 7 | BrainCo / `submit/brainco-hands` | Accepted hand contract; optional vendor SDK. | SDK adapter, absent-SDK behavior, per-side mapping, and separately recorded physical evidence. |
| 8 | Video / `submit/xr-video-adapter` | Agreed XR session and camera contract; no dependency on hands. Keep LeRobot adapter narrow. | Camera freshness, session lifecycle, offscreen pixel test, network path for physical cameras, and manual headset acceptance. |

Video may follow XR immediately if its contract is settled; it must not delay independent hand work. Joystick locomotion and SONIC XR are separate later extensions, not functionality already proven by this series.

The native simulator remains a parallel diagnostic/migration workstream rather than a required upstream feature ancestor.

Proposed ownership must be agreed with each project's maintainers:

| Destination | Responsibility | Avoid duplicating |
|---|---|---|
| LeRobot | Robot/configuration, processors, standard teleoperation/recording integration, optional hand adapter | Independent operator loops and device runtimes |
| Existing Hub simulator | Embodiment models/assets and environment behavior, including agreed G1-23 support | A second permanent production simulator inside the robot driver |
| Isaac Teleop | Shared XR device/session and graphics integration | A second OpenXR lifecycle in LeRobot |
| Operator repository | Installation, pinned launch configurations, acceptance procedures and retained diagnostics | Another independently evolving copy of reusable robot/control code |

The ledger may track contributions to multiple repositories. Maximize upstream-owned functionality, not merely the number of lines merged into LeRobot. If upstream placement is declined or delayed, retain the smallest necessary adapter with explicit ownership and review its maintenance cost at release checkpoints.

## 9. Immediate implementation checkpoint

Execute in order:

1. Completed: create `dev/g1-integration` from `d89a1d0b5f628045cc73293bcdfa2dc6c0808549` and publish it to `MoissanClub/lerobot`. Keyboard submission ref and scratch cleanup are recorded above.
2. Create `docs/delivery-ledger.md` in that branch with rows for the dq fix, G1-23 simulator parity, physical control authority, XR feedback compatibility, and camera delivery. Assign owners and evidence/replacement conditions; do not leave product gaps implicit.
3. Continue the existing dq-fix submission on `g1/bugfixes` (#4664); do not duplicate or rename its active PR head. Verify its actual upstream base before further updates.
4. Verify the #4651 embodiment diff scope against its actual base; disclose inherited keyboard changes until accepted or extracted out. Do not equate dq acceptance with a clean embodiment-only diff.
5. Resolve and test the XR reader feedback-call incompatibility as part of preparing `submit/xr-dual-arm`, before requesting review.

Keep bugfix, keyboard, and embodiments as the preferred early review sequence, while distinguishing historical ancestry from technical prerequisites. Prepare Cartesian on accepted embodiment capabilities and XR on accepted Cartesian capabilities, without native-simulation inheritance. In parallel, start the G1-23 simulator placement discussion and physical command-authority design. Preserve the working pinned product while these submissions evolve.

Obtain focused maintainer feedback on three questions, timed to the PR sequence to avoid blocking early submissions:

1. **Before submitting #4 (Cartesian):** How should the new Cartesian solver share or preserve the existing G1 IK implementation, including moving-base/tool-frame contracts? Open a GitHub Discussion on `huggingface/lerobot` no later than when #3 (embodiments) is in review.
2. **At physical-design kickoff, before enabling hardware:** Should a stock Unitree locomotion/arm-only mode be an additional backend, and how should it coexist with learned-controller modes? Verify authority and compatibility separately for each embodiment. This does not block independent hand composition or simulation PRs.
3. **Before submitting #6 (hands):** What physical end-effector I/O/configuration contract should G1 expose, including body-controller feature overrides and hand fault behavior? Keep this independent of selecting a locomotion backend unless a concrete contract conflict is demonstrated.

Discuss XR graphics/session ownership with Isaac Teleop separately. Keep these discussions narrow enough that they do not block the existing bugfix or keyboard contributions.

## 10. Acceptance and limitations

Every extracted submission must be tested on its intended upstream base. Rerun integration after changing dependencies. Preserve separate evidence for unit tests, Hub/DDS physics, native diagnostic physics, live headset/video, SDK contracts, physical hardware, and fresh installation. Do not reuse an old acceptance label for a rewritten series.

Use [the existing command guide](../../branch-stack-commands.md) and [verification report](../../branch-stack-verification.md) as the starting inventory, not proof for newly extracted submissions. Each ledger row must include: exact checkout SHA, dependency/model pins, working directory and environment setup, executable test/example commands, expected observations or assertions, and the resulting report. Before requesting review, adapt commands to the submission's actual base and include them in its PR description. New Hub Cartesian/XR and physical gates need new evidence; do not relabel existing native-backend tests as coverage for them. Concrete commands for unimplemented gates remain pending, not fabricated runnable instructions.

After every integration update, rerun the affected PR suites and the combined workflow checks. A review-ready device adapter must exercise feedback delivery and processor wiring through the actual standard loop, not only isolated reader/solver tests. Automated headless evidence and recorded manual viewer/headset sign-off are complementary; neither replaces physical acceptance.

### Physical enablement gate

Physical design begins early, but simulation contributions need not wait for hardware validation. Before enabling a physical configuration, name the responsible operator and record the reviewed control-authority mode, validated joint/velocity/torque limits, watchdog location, stale-state/input and disconnect behavior, command handover, and emergency-stop procedure. Follow the manufacturer's applicable bring-up procedure and separately document supervised acceptance for each embodiment and hand configuration. Do not infer hardware readiness from a common action schema, mocked driver, or suspended/fixed-base simulation.

Physical camera acceptance must cover the robot-to-workstation link and frame age through headset delivery. A same-host mmap channel does not supply that network transport. Hand model/tool transforms and compensation assumptions must match the installed hardware; attaching a hand is not only an I/O configuration change.

The reviewed report's 407 cumulative passes were reported results, not rerun here. They do not establish live-headset or physical acceptance. The initial ref-only cleanup is recorded above; source-code extraction, the delivery ledger, new verification gates, and deployment changes remain future work.

## Source basis

This plan is derived from the conversation's inspected sources, not a newly asserted upstream roadmap:

- The discussion referenced `g1_upstream_branch_audit.md`, but that file was not present in this checkout during this revision. Restore or locate it before treating it as reproducible audit evidence; do not infer missing findings from its name.
- Upstream `src/lerobot/robots/unitree_g1/unitree_g1.py`, `config_unitree_g1.py`, `controllers/gr00t_locomotion.py`, `controllers/sonic_whole_body.py`, and `src/lerobot/envs/configs.py`, at the discussed snapshot.
- Upstream `docs/source/unitree_g1.mdx` and `docs/source/isaac_teleop.mdx` inspected during the discussion.
- Upstream commit `77de23890bec5fb3887c5ba8404d6e51be7bfb72`: G1 end-effector and simulation option selection.
- Captured Moissan branches and source files documented in the audit, including `G1CartesianActionProcessor`, `G1ArmKinematics`, `G1Simulation`, `HandSystem`, `HandCollection`, `BrainCoHand`, and XR/video adapters.
- Operator-repository `docs/branch-stack-update-keyboard-20260918.md` and `docs/branch-stack-verification.md`.
- Operator-repository [branch-stack commands](../../branch-stack-commands.md), for the existing tested series rather than future submission claims.
- [Unitree xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate), for the reference feature set, not proof that every embodiment/hand combination has equivalent simulation support.
- [LeRobot G1 documentation](https://github.com/huggingface/lerobot/blob/main/docs/source/unitree_g1.mdx) and [Isaac Teleop](https://github.com/NVIDIA/IsaacTeleop), reviewed during the follow-up: documented SONIC decoder/tracking capabilities do not establish a committed LeRobot roadmap or G1-23 SONIC readiness. These moving sources must be pinned again for implementation decisions.

The central rule is: minimize duplicate implementations and unstable public contracts, not simply the number of branches or the number of lines in a PR.
