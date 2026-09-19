# LeRobot G1 Contribution Branch Plan

## Keyboard-First Review Strategy

This document preserves the tested keyboard-first series. Ongoing development
uses `dev/g1-integration`; this frozen stack is not a prescription to rebuild
all published branches for future submissions. See the
[reference migration](branch-reference-migration-20260918.md) for archived checkpoints.

The September 18 keyboard-first revision demonstrates arm control on upstream's
existing G1-29 simulator before introducing G1-23 or a new simulation backend.
All interactive keyboard control uses `lerobot-teleoperate`, not a new launcher.
The [acceptance commands](branch-stack-commands.md) define exact invocations and
observable goals. The [verification guide](branch-stack-verification.md) records
tested commits and automated results; the [handoff](branch-stack-handoff.md)
identifies the current review candidate and remaining manual work.

Upstream base remains `5aa74557f84c54d4b458f8b9643c5aa2982acfed`.
The frozen combined candidate is `archive/integration/g1-acceptance-keyboard-20260918`.
The installer stays pinned to `d5e400bcefeccc93ba956ce876530e5283512df1`;
automated acceptance does not promote that release. No upstream PR is implied
by creation or local merging of these branches.

## Branches And Acceptance Goals

The table preserves original stage names. Their frozen checkout refs are
`archive/feature-stack-20260918/<original-name>`; the command guide and verifier
use those refs and exact SHAs. Only `g1/bugfixes` and `g1/embodiments` remain
active PR heads. Other original feature names have been retired; future work
uses `dev/g1-integration`, short-lived `work/*`, and curated `submit/*` branches.

| Order | Branch | Required source parent | Verification goal | Suites introduced |
|---|---|---|---|---|
| 0 | `g1/bugfixes` | Pinned upstream base | Correct serialized position-command velocity, gains, torque and neighboring slots. | F |
| 1 | `g1/keyboard-arm-control` | `g1/bugfixes` | Independent G1-29 joint jogging in the existing Hub/DDS simulator, through the standard CLI; no new native simulator or Cartesian code. | K |
| 2 | `g1/embodiments` | `g1/keyboard-arm-control` | G1-29 behavior retained; G1-23 sparse mappings and capability guards; keyboard layout selected from robot configuration. | B |
| 3 | `g1/simulation` | `g1/embodiments` | Same keyboard teleoperator drives every arm joint of either embodiment in local MuJoCo without IK dependencies. | S |
| 4 | `g1/cartesian-control` | `g1/simulation` | FK/IK/gravity and bounded failure behavior; Cartesian action processor and both models' pose sweeps. | C |
| 5 | `g1/xr` | `g1/cartesian-control` | Dual-controller target conversion, independent clutches, tracking recovery and native physics integration. | X |
| 6 | `g1/xr-video` | `g1/xr` | Bounded camera transport, freshness/recovery, shared input/video session and offscreen GPU delivery. | V |
| 7 | `g1/hand-support` | `g1/simulation` | Optional hands under `UnitreeG1`, namespaces, dispatch and failure cleanup, independent of IK/XR. | H |
| 8 | `g1/brainco-hands` | `g1/hand-support` | Explicit BrainCo SDK units, identity, limits, timeout handling and unsupported-simulation rejection. | R |

```text
bugfixes -> keyboard-arm-control -> embodiments -> simulation
                                                   |-> cartesian-control -> xr -> xr-video
                                                   \-> hand-support -> brainco-hands
```

Branch-local suites include all ancestor suites. Sequential-merge acceptance runs
all suites introduced so far, including hands alongside XR at the final stages.
The [runner](../tools/verify_lerobot_branch_stack.py) clones fresh source, verifies
each branch independently, then merges orders 0-8 and retests after every merge.
Required integration tests must pass, not skip. Its scope is the complete targeted
G1 stack, not every unrelated LeRobot dataset/policy/robot test.

## Architecture And Boundaries

- One `UnitreeG1` robot class; G1-29/G1-23 differ by embodiment metadata.
- Keyboard emits named joint targets through the existing teleoperation loop.
  It uses observed positions, bounded discrete increments, model joint limits,
  a target-lead bound, and explicit enable/hold. It does not import a simulator or IK.
- The CLI configures the keyboard layout from `--robot.embodiment`; there is no
  independent keyboard embodiment argument. The initial keyboard branch has only
  the upstream G1-29 definitions. The embodiment branch adds the selection.
- Keyboard CLI acceptance is simulation-only and rejects competing body controllers.
  Joint limits do not prevent self-collision; Space is not a physical emergency stop.
- Native simulation has gravity enabled and optional measured-pose MuJoCo gravity
  feedforward. It is fixed-base, arm-only, collision-free, and in-process, not a
  replacement for the external Hub environment or the experimental DDS runtime.
- Cartesian targets from XR or another source pass through the registered action
  processor and Pinocchio/CasADi solver. Joint control bypasses IK. Cartesian is
  not a runtime requirement of simulation, and the solver does not import MuJoCo.
- Existing upstream exoskeleton IK is retained; the keyboard PR does not replace it.
- Robot cameras feed bounded transport and XR display. Live CloudXR acceptance
  remains separate from headless replay and GPU pixel tests.
- `UnitreeG1Config.hands` composes optional drivers. BrainCo articulated simulation
  is not implemented; physical hand drivers are rejected in simulation.

## Publication And Existing Checkouts

The previous published feature tips are preserved under
`archive/20260918-keyboard/g1/<name>`. Earlier integration branches, including
`archive/integration/g1-acceptance-20260918`, retain their commits for reproducibility.
Publishing the reordered features uses explicit expected-tip force-with-lease,
not unconditional force pushes. `g1/bugfixes` remains unchanged.

Do not merge old feature ancestry into the rebuilt branches. In a clean, idle
checkout, preserve old local names, then use `lerobot-switch` to create the new
tracking branches. Keep unpublished work on its preserved branch and transplant
it deliberately. The active `~/lerobot-dev` checkout is not silently switched.
See [migration details](branch-stack-update-keyboard-20260918.md).

For internal PR verification, create a new target from the pinned upstream base
and merge each branch in order with merge commits; rerun cumulative suites after
every merge. Do not squash shared dependency ancestry. Upstream-facing stacked
PR bases follow the source-parent column. After upstream merges, reassess the
remaining PR bases rather than automatically rewriting shared histories.

## Repository Boundaries

| Location | Responsibility |
|---|---|
| `MoissanClub/lerobot`, locally `../lerobot-upstream` | Native LeRobot contributions and feature branches. |
| This `unitree_g1_lerobot` repository | Operator packaging, experimental runtime, diagnostics, plans and evidence. |
| Fresh acceptance checkout recorded in the verification report | Independent sequential-merge trial, not an editable installation to reuse silently. |
| `../lerobot` | Original editable checkout; preserve existing changes. |

This tracked plan is authoritative. The workspace-root copy is a convenience
mirror. Do not apply experimental embodiment patches to contribution checkouts.

## Remaining Gates

1. User X/keyboard review: both arms/all joints in upstream G1-29, then both native
   embodiments. Check hold/re-enable, limits and shutdown on the recorded commit.
2. User headset review: both embodiments, independent clutches, rotations, tracking
   recovery, video freshness and camera recovery. Prior release reviews do not transfer.
3. Packaging/API/model-provenance review before upstream PRs; explicitly decide
   whether to promote the coworker installer after manual acceptance.
4. Extend external Hub assets separately before claiming G1-23 Hub support.
5. Keep physical G1-23 connections gated and plan hardware/BrainCo validation
   separately. Mocked SDK and simulation tests do not establish physical safety.
6. Continue broader endurance/recovery experiments in the
   [non-physical backlog](future-non-physical-work.md).
