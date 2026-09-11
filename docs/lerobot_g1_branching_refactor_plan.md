# LeRobot G1 Branching and Upstream Refactor Plan

## Purpose

Refactor the current `MoissanClub/unitree_g1_lerobot` work into a clean set of collaborative branches in a fork of `huggingface/lerobot`, while preserving the existing repository as the experimental/integration reference.

The current work has several major threads:

1. G1-23 / G1-29 embodiment support
2. Common Cartesian control / IK / FK / gravity-compensation support
3. G1 simulation improvements
4. G1 XR / VR teleoperation
5. XR camera streaming / OpenXR video feedback
6. Generic hand-system support
7. BrainCo Revo2 hand support

The goal is to separate these into dependency-ordered branches that are easy to review, collaborate on, and eventually submit upstream to Hugging Face LeRobot.

---

## High-Level Repository Strategy

Use two repositories with different roles.

### `MoissanClub/unitree_g1_lerobot`

Purpose:

- experimentation
- end-to-end integration
- diagnostics
- acceptance testing
- temporary adapters
- comparison scripts
- physical robot tests
- CloudXR / OpenXR experiments
- rapid development

Question this repo answers:

> Does the complete G1 system work?

Do not rewrite or destroy the current history.

### `MoissanClub/lerobot`

Create this as a fork of:

```text
huggingface/lerobot
```

Purpose:

- clean LeRobot-native implementation
- collaboration between MoissanClub developers
- upstream-quality branches
- tests and documentation
- staging area for Hugging Face PRs

Question this repo answers:

> Is this a clean, reusable LeRobot contribution?

---

# Verification Milestones and PR Acceptance

These milestone numbers are independent of the procedural step numbers below.
Each feature PR must deliver its named suites, setup instructions, and verification
commands with its implementation. Only milestone 1's suites currently exist in the
fork. All other paths in this table are planned deliverables, not existing tests.

| Milestone and verification goal | Branch created | Previous milestone dependency | Test suites to run |
|---|---|---|---|
| 0. Prepare an isolated fork checkout; preserve the working environment. | None; prepare `main` and remotes. | None. | Check origin/upstream URLs, recorded upstream base, clean fork checkout, and preserved reference repositories/environment. Completed. |
| 1. Verify both embodiments' configuration, sparse joint mapping, action/observation schemas, and early rejection of unsupported connections. Preserve G1-29 behavior. | `g1/embodiments` | 0; branch from fork `main`. | Common regression suite **B** below. Completed at `ec59825d`: 128 passed, 1 skipped, including the optional SONIC module. |
| 2. Verify FK, IK, joint limits, continuity, and gravity feedforward using each embodiment's selected model. | `g1/cartesian-control` | 1; branch from `g1/embodiments`. | **B** + planned `tests/robots/test_unitree_g1_cartesian_control.py` and `tests/robots/test_unitree_g1_kinematics.py`. Side-by-side kinematic playback of translations and hand rotations with numerical results; no motor dynamics required yet. |
| 3. Run both embodiments through LeRobot and MuJoCo with correct commands, feedback, motor response, and camera output. | `g1/simulation` | 2; branch from `g1/cartesian-control` for shared control/gravity verification. | Milestone 2 suites + planned `tests/robots/test_unitree_g1_simulation.py` and `tests/integration/test_unitree_g1_mujoco_runtime.py`. Headless runtime plus visible keyboard/scripted motion for both embodiments, gravity compensation on/off. |
| 4. Independently control both arms through XR, including clutch, release, tracking loss, and reconnection. | `g1/xr` | 2 for source; 3 additionally for simulated acceptance. Branch from `g1/cartesian-control`. | Milestone 2 suites + planned `tests/teleoperators/test_unitree_g1_xr.py`. On the combined acceptance branch, also milestone 3 suites and `tests/integration/test_unitree_g1_xr_mujoco.py`. Synthetic replay followed by headset review for both embodiments. |
| 5. See live robot-camera images while controlling either embodiment; recover from stream interruption. | `g1/xr-video` | 4 for source; 3 additionally for simulated acceptance. Branch from `g1/xr`. | Milestone 4 suites + planned `tests/teleoperators/test_unitree_g1_xr_video.py`; on the combined acceptance branch, also `tests/integration/test_unitree_g1_xr_video_mujoco.py`. Check changing frames, timestamps, freshness, recovery, and headset view during arm motion. |
| 6. Compose optional hands with the body through one interface without XR or a vendor SDK. | `g1/hand-support` | 2; branch from `g1/cartesian-control`. | Milestone 2 suites + planned `tests/robots/test_unitree_g1_hands.py`. Fake-hand tests for namespaced features, dispatch, observations, lifecycle, failure cleanup, and unchanged no-hand operation. |
| 7. Map canonical hand actions and SDK feedback correctly for both BrainCo hands. | `g1/brainco-hands` | 6; branch from `g1/hand-support`. | Milestone 6 suites + planned `tests/robots/test_unitree_g1_brainco_hands.py`. SDK-mocked tests/replay for units, limits, side mapping, malformed feedback, and disconnects. Physical motion/tactile acceptance remains a separate hardware gate. |

## Common Regression Suite B

From the checked-out fork with the documented dependencies installed:

```bash
PYTHONPATH=src python -m pytest -q \
  tests/robots/test_unitree_g1.py \
  tests/robots/test_unitree_g1_utils.py \
  tests/robots/test_unitree_g1_embodiments.py \
  tests/teleoperators/test_unitree_g1_teleoperator.py \
  tests/robots/test_sonic_whole_body.py
```

For later milestones, append the named paths and their dependency suites. Every
PR must document a copy-pastable cumulative command, dependency installation, and
any visual verification command. Missing suites, unavailable required dependencies,
or skipped required tests do not count as passing. Report optional unrelated
controller skips separately.

Tests must import the checked-out fork, not the old editable installation.
Non-physical tests must not discover/connect to physical robots; isolate DDS
domains, ports, and temporary resources. The experimental repository's old tests
cannot by themselves establish that the port works.

Define numerical tolerances in the suites before declaring success: pose error,
joint limits, continuity, settling behavior, feedforward error, or frame age as
appropriate. Test reachable and unreachable targets explicitly. G1-23 need not
reproduce every G1-29 hand pose; its position/orientation constraints and bounded
failure behavior must be verified. Record commands, commits, model revisions,
results, and manual observations with the acceptance evidence.

## Checkout, PR, Merge, and Retest

Keep fork `main` tracking Hugging Face. Use a separate cumulative acceptance branch
for internal feature merges rather than merging the feature stack into `main`.

1. Fetch `origin`, check out each feature branch in an isolated checkout/worktree,
   and run its branch-local suites. Review the diff against its declared parent.
2. Create and push `integration/g1-acceptance` from the recorded fork `main` base.
   This is an internal PR target, not a feature branch for upstream submission or
   the source base for new feature work.
3. Open and merge one internal PR per milestone into `integration/g1-acceptance`,
   in order 1 through 7. Preserve ancestry using merge commits while branches are
   stacked; do not squash/rebase-merge shared dependency commits into this branch.
   Review any sibling conflicts and ensure later PR diffs contain only unmerged
   work.
4. After each merge, fetch/check out the acceptance branch and rerun all suites
   belonging to merged milestones. At milestones 4 and 5 it contains simulation
   plus XR, so run their actual headless integration suites there. XR branch-local
   tests use fake input/robot boundaries; simulation is not an artificial source
   dependency of XR.
5. Complete required visual/headset review and record the tested merge commit.
   Headless results do not substitute for headset acceptance. Milestone 7 may
   pass its non-physical gate without claiming hardware validation.

After each merged PR you have a branch available from `origin` with a cumulative
verification suite. Keep feature branches separately reviewable for upstream PRs.
Use a new acceptance branch for a substantially rebased stack rather than
force-rewriting reviewed integration history.

---

# Phase 1 — Create and Prepare the MoissanClub LeRobot Fork

## Step 1 — Fork Hugging Face LeRobot

Fork:

```text
https://github.com/huggingface/lerobot
```

into:

```text
https://github.com/MoissanClub/lerobot
```

Do not use `unitree_g1_lerobot` as the upstream fork.

---

## Step 2 — Clone the MoissanClub fork

```bash
git clone git@github.com:MoissanClub/lerobot.git
cd lerobot
```

Configure Hugging Face as the upstream remote:

```bash
git remote add upstream https://github.com/huggingface/lerobot.git
git fetch upstream
```

Verify:

```bash
git remote -v
```

Expected mental model:

```text
origin/main
    = MoissanClub/lerobot

upstream/main
    = huggingface/lerobot
```

---

## Step 3 — Keep `MoissanClub/lerobot:main` clean

Do not develop directly on `main`.

Synchronize it with Hugging Face:

```bash
git checkout main
git fetch upstream
git merge --ff-only upstream/main
git push origin main
```

Rule:

> `MoissanClub/lerobot:main` should closely track `huggingface/lerobot:main`.

---

# Phase 2 — Preserve the Existing Experimental Repository

## Step 4 — Preserve the current integrated state

Do not rewrite:

```text
MoissanClub/unitree_g1_lerobot:main
```

Optionally tag the current state:

```bash
git checkout main
git pull origin main

git tag pre-upstream-reorg-2026-09-10
git push origin pre-upstream-reorg-2026-09-10
```

This repository remains the source of working/proven changes that will be selectively ported into the clean LeRobot fork.

---

# Phase 3 — Reconstruct Clean Feature Branches

The branch structure should follow software dependencies, not the chronological order in which the features were originally developed.

Target branch DAG:

```text
huggingface/lerobot:main
          │
          ▼
MoissanClub/lerobot:main
          │
          ▼
    g1/embodiments
          │
          ▼
 g1/cartesian-control
      /       |        \
     /        |         \
    ▼         ▼          ▼
g1/simulation g1/xr   g1/hand-support
                │          │
                ▼          ▼
           g1/xr-video  g1/brainco-hands
```

For this verification plan, simulation branches from `g1/cartesian-control` so its
acceptance covers the shared control and gravity-compensation implementation.

---

# Phase 4 — Build `g1/embodiments`

## Step 5 — Create the embodiment branch

```bash
git checkout main
git pull origin main

git checkout -b g1/embodiments
```

Port only the changes required for:

```text
G1-29
+
G1-23
+
common embodiment configuration
+
joint definitions / indexing / mapping
+
model / URDF selection required by embodiment
```

Do not include:

- XR code
- CloudXR code
- video streaming
- BrainCo
- generic hand support
- one-off simulation diagnostics
- comparison screenshots/tools unless necessary to LeRobot itself

Use the current experimental repo as a reference/source.

Possible techniques:

```bash
git checkout <experimental-branch-or-commit> -- path/to/file
```

or cherry-pick clean commits when they map exactly to this feature:

```bash
git cherry-pick <commit-sha>
```

In practice, manual selective porting may be cleaner than cherry-picking because the existing history contains integrated changes.

After porting:

```bash
git diff main
```

Review criterion:

> Does this diff describe exactly “support configurable G1-23 and G1-29 embodiments”?

Commit in logical pieces:

```bash
git add ...
git commit -m "Add configurable G1-23 and G1-29 embodiments"
```

Push:

```bash
git push -u origin g1/embodiments
```

---

# Phase 5 — Extract Common Cartesian / Kinematics Control

## Step 6 — Create the common Cartesian-control branch

```bash
git checkout g1/embodiments
git checkout -b g1/cartesian-control
```

Port reusable G1 control infrastructure that should be shared by:

- XR teleoperation
- policy rollout
- simulation
- future teleoperators
- future Cartesian-action policies

This branch should contain reusable pieces such as:

```text
FK
IK
Cartesian target → joint target
joint ordering / mapping
kinematic model selection
gravity-compensation helpers
common arm-control helpers
safety/range helpers if generally applicable
```

Important design goal:

```text
policy / teleop
      ↓
canonical Cartesian target
      ↓
common G1 Cartesian-control layer
      ↓
G1 joint target
```

Avoid keeping reusable IK/FK inside XR-specific files.

Review:

```bash
git diff g1/embodiments
```

Criterion:

> Does this branch contain only generic G1 Cartesian/kinematics/control capability?

Push:

```bash
git push -u origin g1/cartesian-control
```

---

# Phase 6 — Extract Simulation as a Sibling Workstream

## Step 7 — Choose the minimum correct simulation base

Use common Cartesian control as the parent for this plan's acceptance scope:

```bash
git checkout g1/cartesian-control
git checkout -b g1/simulation
```

Port only reusable LeRobot simulation capability.

Good upstream candidates:

- real/sim embodiment consistency
- G1-23 simulation support
- reusable MuJoCo interfaces
- camera support that belongs naturally in LeRobot simulation
- consistent action/observation semantics
- tests

Likely keep in `unitree_g1_lerobot`:

- comparison screenshots
- manual acceptance scripts
- one-off debugging launchers
- narrow developer diagnostics
- experimental visualization tools

Review:

```bash
git diff <simulation-parent-branch>
```

Push:

```bash
git push -u origin g1/simulation
```

---

# Phase 7 — Extract XR / VR Teleoperation

## Step 8 — Create the XR control branch

```bash
git checkout g1/cartesian-control
git checkout -b g1/xr
```

Port:

- XR controller acquisition
- controller pose handling
- coordinate-frame conversion
- clutching / rebasing
- dual-arm target generation
- integration with generic G1 Cartesian control
- XR-specific configuration

Architecture should be:

```text
XR input
   ↓
canonical Cartesian target
   ↓
common G1 Cartesian control
   ↓
joint targets
   ↓
UnitreeG1
```

Do not duplicate IK/FK inside XR if it can live in the shared control layer.

Push:

```bash
git push -u origin g1/xr
```

---

# Phase 8 — Separate XR Camera / Video Feedback

## Step 9 — Create a child XR-video branch

```bash
git checkout g1/xr
git checkout -b g1/xr-video
```

Port only video/XR feedback functionality, such as:

- MuJoCo head-camera output
- camera transport
- Isaac Teleop camera-streaming integration
- Televiz / `camera_viz`
- OpenXR quad/video surface
- CloudXR video feedback
- headset display verification

Keep this separate from basic XR control because the dependency footprint and review surface are much larger.

Architecture:

```text
control:
Quest/XR → G1

video:
G1 or MuJoCo camera → XR/OpenXR → Quest
```

Push:

```bash
git push -u origin g1/xr-video
```

---

# Phase 9 — Add Generic Hand-System Composition

## Step 10 — Create generic G1 hand support

Do not branch hand support from XR.

Hands must also work for:

- model rollout
- dataset recording
- scripted control
- future non-XR teleoperators

Branch from common G1 control:

```bash
git checkout g1/cartesian-control
git checkout -b g1/hand-support
```

Add the generic composition mechanism.

Target idea:

```text
UnitreeG1
├── G1 body / arms
└── hands: HandSystem | None
```

or equivalent.

The generic hand layer should support future implementations such as:

```text
BrainCoHands
Dex3Hands
InspireHands
```

The branch should define:

- hand configuration interface
- optional hand lifecycle
- observation feature composition
- action feature composition
- `get_observation()` merge semantics
- `send_action()` split/dispatch semantics
- left/right naming convention
- simulation/no-hand compatibility where relevant

Avoid BrainCo-specific SDK code in this branch if practical.

Review:

```bash
git diff g1/cartesian-control
```

Criterion:

> Does this branch add generic optional hand composition without depending on BrainCo?

Push:

```bash
git push -u origin g1/hand-support
```

---

# Phase 10 — Add BrainCo Revo2 as a Hand Implementation

## Step 11 — Create BrainCo support

```bash
git checkout g1/hand-support
git checkout -b g1/brainco-hands
```

Add BrainCo-specific implementation:

- connection lifecycle
- serial / SDK integration
- left and right hand support
- motor command mapping
- motor observations
- tactile observations
- closure or dexterous retargeting
- configuration
- safety limits / rate limits appropriate to the driver
- tests

Architecture:

```text
canonical hand action
       ↓
hand retargeting / closure mapping
       ↓
BrainCoHands
       ↓
BrainCo SDK / serial
```

Push:

```bash
git push -u origin g1/brainco-hands
```

---

# Phase 11 — Continue Rapid Local Development Separately

## Step 12 — Keep using `unitree_g1_lerobot` for experiments

New experimental work should continue in:

```text
MoissanClub/unitree_g1_lerobot
```

using normal development branches, for example:

```bash
git checkout main
git checkout -b dev/brainco
```

or:

```bash
git checkout -b dev/physical-g1
git checkout -b dev/xr-video
```

When proven:

```text
dev/*
  ↓
unitree_g1_lerobot/main
```

Then separately extract/port the clean reusable portion into:

```text
MoissanClub/lerobot
```

Do not force the experimental repo to have the same commit history as the upstream staging fork.

---

# Phase 12 — Collaboration Workflow in `MoissanClub/lerobot`

## Step 13 — Collaborators branch from the appropriate feature base

Example: collaborator working on BrainCo:

```bash
git clone git@github.com:MoissanClub/lerobot.git
cd lerobot

git remote add upstream https://github.com/huggingface/lerobot.git
git fetch origin
git fetch upstream

git checkout g1/hand-support
git checkout -b dev/<name>-brainco
```

Collaborator opens an internal PR:

```text
dev/<name>-brainco
        ↓
g1/brainco-hands
```

This allows MoissanClub review and CI before any Hugging Face upstream submission.

---

# Phase 13 — Upstream PR Strategy

## Step 14 — Submit PRs in dependency order

Likely order:

```text
PR 1: g1/embodiments
PR 2: g1/cartesian-control
PR 3: g1/simulation
PR 4: g1/xr
PR 5: g1/xr-video
PR 6: g1/hand-support
PR 7: g1/brainco-hands
```

Exact order of sibling PRs can vary.

Important:

- `g1/simulation`
- `g1/xr`
- `g1/hand-support`

are conceptually siblings after their common dependency.

They do not need to wait on each other.

---

## Step 15 — Use stacked PRs while earlier PRs are pending

Do not wait months for PR 1 to merge before preparing PR 2.

Example:

```text
PR 1:
MoissanClub:g1/embodiments
    → huggingface:main

PR 2 while PR 1 is pending:
MoissanClub:g1/cartesian-control
    → MoissanClub:g1/embodiments
```

This makes the PR 2 diff show only the additional Cartesian-control changes.

Likewise:

```text
g1/xr
    → g1/cartesian-control

g1/hand-support
    → g1/cartesian-control

g1/brainco-hands
    → g1/hand-support
```

Use draft PRs where helpful.

---

# Phase 14 — Rebase After Upstream Merges

## Step 16 — Sync the Moissan fork after an upstream merge

If Hugging Face merges `g1/embodiments`:

```bash
git checkout main
git fetch upstream

git reset --hard upstream/main
git push --force-with-lease origin main
```

`MoissanClub/lerobot:main` should again match upstream.

---

## Step 17 — Rebase descendants so their PR diff becomes clean

Example for Cartesian control:

```bash
git checkout g1/cartesian-control

git rebase --onto main g1/embodiments

git push --force-with-lease origin g1/cartesian-control
```

After Cartesian control merges, repeat for:

```text
g1/xr
g1/hand-support
g1/simulation
```

depending on their parent.

For example:

```bash
git checkout g1/xr
git rebase g1/cartesian-control
git push --force-with-lease origin g1/xr
```

Always use:

```bash
git push --force-with-lease
```

rather than unrestricted `--force`.

---

# Phase 15 — Branch Review Rule

For every upstream-staging branch, run:

```bash
git diff <parent-branch>...HEAD
```

and ask:

> Does this diff contain exactly one conceptual feature?

Examples:

```text
g1/embodiments
    = only embodiment support

g1/cartesian-control
    = only reusable control/kinematics

g1/simulation
    = only reusable simulation changes

g1/xr
    = only XR control

g1/xr-video
    = only XR camera/video

g1/hand-support
    = only generic optional-hand architecture

g1/brainco-hands
    = only BrainCo implementation
```

If unrelated functionality appears in a diff, move/refactor it into its proper parent or sibling branch.

---

# Phase 16 — Important Architectural Rules

## Rule 1 — Software dependency order, not historical commit order

Do not reconstruct branches by replaying the old commits chronologically.

The existing history reflects discovery and experimentation.

The clean branch graph should reflect:

```text
what depends on what
```

---

## Rule 2 — Keep generic functionality below feature-specific integrations

Example:

Bad:

```text
XR module
    contains G1 IK
```

Better:

```text
common G1 Cartesian control
    contains IK

XR
    consumes Cartesian control

policy rollout
    consumes Cartesian control
```

---

## Rule 3 — Hand support must not depend on XR

Correct:

```text
                   G1 Cartesian control
                    /              \
                   /                \
                  ▼                  ▼
                XR             hand support
                                   ↓
                                BrainCo
```

Not:

```text
XR
 ↓
hands
 ↓
BrainCo
```

because policy rollout and recording also need hand support.

---

## Rule 4 — Keep hardware drivers below canonical model/teleop representations

Preferred architecture:

```text
ACT / VR
   ↓
canonical action
   ↓
robot action processors
   ├── arm IK
   └── hand retargeting
   ↓
UnitreeG1 composite
   ├── G1 arm/body driver
   └── hand driver
```

---

## Rule 5 — Preserve experimental tooling where it belongs

Not everything in `unitree_g1_lerobot` needs to move upstream.

Keep experimental-only tools there unless they clearly provide reusable LeRobot value.

Examples:

- comparison scripts
- physical acceptance checklists
- temporary launchers
- narrow diagnostics
- investigation utilities

---

# Final Execution Order

Codex should perform the repository reconstruction in this order:

```text
1. Verify/create MoissanClub/lerobot fork.

2. Clone/use MoissanClub/lerobot.

3. Configure:
       origin   = MoissanClub/lerobot
       upstream = huggingface/lerobot

4. Synchronize origin/main with upstream/main.

5. Preserve MoissanClub/unitree_g1_lerobot unchanged as reference.

6. Create g1/embodiments.
       Port only G1-23/G1-29 embodiment support.

7. Create g1/cartesian-control from g1/embodiments.
       Port/extract shared IK/FK/control functionality.

8. Create g1/simulation from g1/cartesian-control.
       Port reusable simulation changes.

9. Create g1/xr from g1/cartesian-control.
       Port XR controller/teleop functionality.

10. Create g1/xr-video from g1/xr.
        Port camera/OpenXR/CloudXR functionality.

11. Create g1/hand-support from g1/cartesian-control.
        Implement generic optional hand-system composition.

12. Create g1/brainco-hands from g1/hand-support.
        Add BrainCo Revo2 support.

13. Run tests and inspect each parent...child diff.

14. Push every clean branch to MoissanClub/lerobot.

15. Use internal MoissanClub PRs for collaborative review.

16. Open Hugging Face upstream PRs in dependency order.

17. Keep developing experimental end-to-end work in
        MoissanClub/unitree_g1_lerobot.

18. When upstream PRs merge:
        sync MoissanClub/lerobot main,
        rebase descendant branches,
        force-push with --force-with-lease.

19. Gradually remove duplicated experimental code only after
        equivalent functionality is stable upstream.
```

---

# Safety / Preservation Requirements for the Refactor

Codex should follow these constraints:

1. Do not rewrite or delete the current `unitree_g1_lerobot/main` history.
2. Do not force-push existing experimental branches unless explicitly requested.
3. Create the clean branch structure in `MoissanClub/lerobot`, not by destructively transforming the existing repo.
4. Prefer selective porting over broad cherry-picks when commits contain mixed concerns.
5. Before each commit, inspect the diff against its intended parent.
6. Keep each upstream branch buildable/testable where practical.
7. Preserve backward compatibility for existing G1-29 behavior whenever possible.
8. Do not make XR a dependency of policy rollout, hand support, or generic G1 control.
9. Do not make BrainCo-specific code part of the generic hand abstraction unless unavoidable.
10. Use `--force-with-lease`, never unrestricted `--force`, when rebasing published branches.

---

# Desired End State

At the end of the refactor:

```text
MoissanClub/unitree_g1_lerobot
    = working experimental/integration environment

MoissanClub/lerobot
    main
    g1/embodiments
    g1/cartesian-control
    g1/simulation
    g1/xr
    g1/xr-video
    g1/hand-support
    g1/brainco-hands

huggingface/lerobot
    = eventual upstream destination
```

The branch graph should make it possible to upstream each capability independently while continuing full-speed local G1 development.
