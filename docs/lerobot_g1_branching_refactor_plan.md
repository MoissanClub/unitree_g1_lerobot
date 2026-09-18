# LeRobot G1 Contribution Branch Plan

## Current Checkpoint

As of 2026-09-17, the bug-fix branch and seven feature branches are committed and
pushed to `MoissanClub/lerobot`. All eight branch-local suites and all eight
sequential merges passed headlessly. See the [verification guide](branch-stack-verification.md)
for exact results, tested tips and reproduction commands.

- Upstream base and fork `main`: `5aa74557f84c54d4b458f8b9643c5aa2982acfed`.
- Combined contribution branch: `integration/g1-acceptance-20260917` at
  `da1c0fcd5f6936560536e93af7e1528dbaaf75be`.
- Installed operator release: still pinned to `d5e400bcefeccc93ba956ce876530e5283512df1`.
  Do not change that pin before reviewing the new stack with X and the headset.
- No GitHub PRs have been opened or merged for this revision. Local sequential
  merge verification is not upstream acceptance.

This is the authoritative dependency and contribution plan. The [handoff](branch-stack-handoff.md)
contains the next actions; dated changes are in the [September 17 revision note](branch-stack-update-20260917.md).
The [superseded reconstruction plan](archive/branch-stack-pre-cleanup-20260917/lerobot_g1_branching_refactor_plan.md)
is retained as history, not as instructions to recreate completed branches.

## Repository Boundaries

| Repository or checkout | Responsibility |
|---|---|
| `MoissanClub/lerobot`, locally `../lerobot-upstream` | LeRobot-native contributions and the dependency-ordered branch stack. |
| `MoissanClub/unitree_g1_lerobot`, this repository | Operator packaging, experimental runtime, diagnostics, integration evidence and this plan. |
| `../lerobot-acceptance-20260917-v6` | Fresh-clone sequential-merge review checkout; tree matches the published cumulative branch. |
| `../lerobot` | Original editable checkout; preserve its existing modifications. |

Paths are relative to this repository. The workspace-root copy of this plan is
a convenience mirror; this tracked document is authoritative. Do not apply the
experimental embodiment patch to contribution or review checkouts.

## Branches And Verification Goals

Suite identifiers below link to the [test suite definitions](branch-stack-verification.md#test-suites).
Branch-local suites include their prerequisites; cumulative verification runs
all suites introduced so far plus the relevant cross-branch integration tests.

| Order | Branch | Required source parent | Concrete verification goal | Branch-local suites |
|---|---|---|---|---|
| 0 | `g1/bugfixes` | Fork `main` at the pinned base | Serialized position commands clear the SDK `dq` field and preserve correct targets, gains, torque and neighboring slots. No embodiment feature changes. | F |
| 1 | `g1/embodiments` | `g1/bugfixes` | Correct G1-23 sparse mapping, G1-29 defaults, schemas, capability rejection and typed Hub configuration. | B |
| 2 | `g1/cartesian-control` | `g1/embodiments` | FK/IK/gravity correctness, bounded failures, joint ordering and the registered Cartesian action processor. | B + C |
| 3 | `g1/simulation` | `g1/cartesian-control` | Both embodiments move and return feedback in native MuJoCo; gravity on/off and changing camera frames work. Unsupported diagnostic hand models fail explicitly. | B + C + S |
| 4 | `g1/xr` | `g1/cartesian-control` | Independent clutches, timestamps, tracking loss and re-engagement; controller targets pass through the IK action processor. | B + C + X |
| 5 | `g1/xr-video` | `g1/xr` | Bounded camera transport, freshness/recovery and shared input/video session; offscreen GPU pixels are verified. | B + C + X + V |
| 6 | `g1/hand-support` | `g1/simulation` | Optional hands compose under `UnitreeG1`; namespaces, dispatch, validation, body homing and failure cleanup work without changing no-hand behavior. | B + C + S + H |
| 7 | `g1/brainco-hands` | `g1/hand-support` | Explicit BrainCo configuration, SDK units/limits/identity, timeout handling and rejection of unsupported simulation. | B + C + S + H + R |

XR/video remain independent of simulation in source ancestry. Their combined
MuJoCo suites run after merging simulation. Hand support depends on simulation
because both modify the robot lifecycle; those conflicts are resolved in the
hand branch rather than left to every reviewer.

The standalone bug fix precedes the embodiment feature in the PR sequence.
Existing histories were preserved with ordinary merges, including the user's
September 15 updates. The original embodiment commit still contains the fix
historically, but the feature diff against `g1/bugfixes` does not.

## Architecture

```text
XR poses/buttons/timestamps
  -> tracking validation, frame conversion and independent clutches
  -> registered G1 Cartesian action processor
  -> bounded numerical IK
  -> named joint commands
  -> UnitreeG1: selected embodiment, execution backend and optional hands

Robot cameras -> bounded image transport -> XR video delivery
```

- One `UnitreeG1` identity, with embodiment data rather than G1-29/G1-23 subclasses.
- The Pinocchio/CasADi solver remains independent of XR and processors.
  `G1CartesianActionProcessor` consumes root-frame pose matrices and measured
  joint observations and emits named `.q` radians. Intermediate pose matrices
  are not a newly defined flat dataset schema.
- Gravity compensation belongs to execution/dynamics, not the XR input reader.
  Native simulation uses measured-pose feedforward; this does not certify the
  physical model or hand/payload compensation.
- Preserve upstream `G1EndEffector` and `UnitreeG1MujocoEnv` configuration.
  Hub simulation retains upstream end-effector/camera options. The explicit local
  URDF simulator is a fixed-base diagnostic backend, not a replacement Hub task.
- `UnitreeG1Config.hands` composes optional drivers without a second body publisher.
  `G1WithHands` is compatibility-only. BrainCo uses `end_effector="brainco"`
  and explicit hand configurations; vendor SDKs remain optional.
- XR/video are robot-independent where possible. Shared adapter placement with
  the upstream Isaac example remains a maintainer discussion.
- Arm teleoperation must respect any body controller's ownership. G1-29
  controller support does not imply G1-23 controller compatibility.

## PR And Collaboration Workflow

1. Review each feature against its required source parent in the table.
   While dependencies are unmerged upstream, use stacked PR bases.
2. For an internal acceptance trial, create a new branch from the pinned upstream
   base and merge orders 0-7 with merge commits. Do not target the already-combined
   acceptance branch or squash shared dependency ancestry.
3. Run each stage's cumulative suites after its merge. The
   [verification runner](../tools/verify_lerobot_branch_stack.py) automates a fresh
   clone, branch tests, sequential merges and headless examples; it does not open
   GitHub PRs or push.
4. Use `integration/g1-acceptance-20260917` for combined development/review.
   Keep feature changes on the appropriate branch and rerun affected descendants
   and cumulative tests before publishing.
5. After actual upstream merges, reassess descendant PR bases and diffs.
   Do not automatically rebase or force-push shared branches. Fork `main` stays
   upstream-only; never merge the feature stack into it.

Original tips remain in local `archive/20260917/*` and
`archive/20260917/origin/*` backup branches. The older
`integration/g1-acceptance` remote and installer pin are preserved.

## Remaining Gates

1. Review X viewers and headset control/video for both embodiments on the new
   cumulative commit, then decide whether to promote the installer pin.
2. Review packaging, model provenance/licensing and public APIs before upstream PRs.
3. Extend the external Hub model repository before claiming G1-23 or BrainCo Hub
   support. BrainCo articulated simulation is not implemented; physical hand
   drivers are rejected in simulation.
4. Keep physical G1-23 connection gated. Physical G1 and BrainCo acceptance must
   verify mounting/model, motor anatomy, limits, payload effects and stop behavior.
   Mocked SDK tests do not prove these.
5. Continue broader workspace, recovery, endurance and native DDS cleanup work in
   the [non-physical backlog](future-non-physical-work.md). Branch regression
   completion does not close those experiments.
