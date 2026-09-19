# G1 Branch Stack Verification

## Keyboard-First Checkpoint

Frozen checkpoint: `archive/integration/g1-acceptance-keyboard-20260918`.
Ongoing development: `dev/g1-integration`. Historical machine-readable reports
retain the branch names used at execution time; use the exact recorded SHAs and
the [reference mapping](branch-reference-migration-20260918.md), not a moving
development tip, to reproduce those results.
Upstream base: `5aa74557f84c54d4b458f8b9643c5aa2982acfed`.
The [branch plan](lerobot_g1_branching_refactor_plan.md) owns dependencies;
[acceptance commands](branch-stack-commands.md) owns the exact per-stage pytest
and interactive commands. The [revision note](branch-stack-update-keyboard-20260918.md)
explains migration from existing local branches.

The nine branch-local and nine sequential-merge stages are recorded in the
[machine-readable report](verification/lerobot-branch-stack-keyboard-20260918.json).
Its `branch_commits` identify tested feature tips, and `final_commit` identifies
the combined branch produced by the fresh sequential merge trial.

| Order | Branch | Tested tip | Branch-local passes | Cumulative passes |
|---|---|---|---:|---:|
| 0 | `g1/bugfixes` | `e272f385` | 96 | 96 |
| 1 | `g1/keyboard-arm-control` | `2343d572` | 169 | 169 |
| 2 | `g1/embodiments` | `50e0b691` | 239 | 239 |
| 3 | `g1/simulation` | `987ff6ec` | 266 | 266 |
| 4 | `g1/cartesian-control` | `c1c8435a` | 322 | 322 |
| 5 | `g1/xr` | `101a9964` | 349 | 349 |
| 6 | `g1/xr-video` | `44a34f25` | 362 | 362 |
| 7 | `g1/hand-support` | `abcc1d67` | 281 | 377 |
| 8 | `g1/brainco-hands` | `476a8452` | 311 | 407 |

Acceptance requires no unexpected skips. The video branch and cumulative stages
6-8 additionally run one offscreen Vulkan/CUDA pixel test each. Lint/format,
camera-channel typing, installed SDK contracts and both embodiments' headless
examples are included. The source clone is fresh; the Python environment is reused,
so this is not clean-OS installation proof or the entire unrelated LeRobot suite.
X, live headset and physical acceptance are not claimed.

Full logs and JUnit XML are retained locally in
`outputs/branch-stack-keyboard-20260918-final/`. The first attempt, retained in
`outputs/branch-stack-keyboard-20260918/`, passed through XR but failed the GPU
gate because Vulkan could not access a suitable device inside the sandbox.
The isolated GPU gate passed outside the sandbox; the final full trial uses that
execution environment rather than bypassing or skipping GPU verification.

## What The Keyboard Gates Exercise

An additional check in the existing `lerobot-dev` conda environment passed all
51 keyboard unit, native simulator, Hub/DDS and terminal CLI tests (zero skips)
at acceptance commit `d89a1d0b5f628045cc73293bcdfa2dc6c0808549`.
Its JUnit report is retained at
`outputs/branch-stack-keyboard-20260918-final/lerobot-dev-keyboard.xml`.
The installed Unitree SDK wheel lacked its native CRC library; restoring package
data from the exact installed SDK revision resolved that environment failure.
No user checkout, package version or editable-source pointer was changed.

- Pure keyboard contracts and shared upstream listener regression tests.
- G1-29 real Hub/DDS physics, synthetic discrete input through the standard
  teleoperation loop, and measured movement of both arms.
- Actual CLI parsing and terminal input through a POSIX PTY, with no X listener.
- G1-29/G1-23 native physics: every arm joint, isolated targets, model limits,
  correct directions and measured tracking, plus both real CLI variants.
- Measured-pose initialization, enable/hold, rate limiting, no queued replay,
  stale/missing/invalid feedback, listener failure, joint bounds and target lead.

The Hub fixture pins `68459ed68f6f68e1f661091dfcb6ebce44681aec`.
The ordinary CLI resolves the configured Hub default; the final trial uses the
cache containing that revision with `HF_HUB_OFFLINE=1`. Tests do not patch the
external simulator's physics or replace DDS transport. Hub tests run serially.

The motor-command regressions also use real SDK messages at the serialization
boundary, not only mocks. They require Unitree SDK/CycloneDDS and must not skip.
Hand SDK tests establish driver contracts, not physical safety.

## Reproduce The Full Trial

From this operator repository, with the existing test environment available:

```bash
export CYCLONEDDS_HOME=/home/dwei/lerobot-dev/environment/native/5041f3560c088c99e5088b2b8520b69169621196/install
export HF_HOME=/home/dwei/lerobot-sim/.cache/hf-g1
export HF_HUB_OFFLINE=1
python tools/verify_lerobot_branch_stack.py \
  --remote git@github.com:MoissanClub/lerobot.git \
  --checkout ../lerobot-keyboard-review-fresh \
  --output outputs/keyboard-review-fresh \
  --assets ../.cache/g1-cartesian-assets \
  --python /home/dwei/lerobot-sim/.venvs/lerobot-upstream/bin/python \
  --sdk-site-packages /home/dwei/.venvs/isaacteleop/lib/python3.12/site-packages \
  --gpu --integration-branch integration/g1-keyboard-pr-review
```

Use new checkout/output paths for each run. The paths above are this workstation's
setup: replace them on another host. Offline mode requires the Hub model already
cached; omit it to download dependencies beforehand. Native CycloneDDS must be ABI
compatible with its Python bindings; this workstation's older native library can
abort in SDK tracing before control begins. `lerobot-dev` selects the tested build.
The report records these environment selectors for reproducibility.

The runner checks source import provenance, enables all required G1 integration
flags, tests each branch, merges all nine stages into a new branch, and repeats
cumulative tests after every merge. It never pushes, opens PRs, resets existing
checkouts or contacts physical hardware. Without `--gpu`, its status is
`passed_without_gpu_gate`, not complete acceptance.

## Manual PR And Visual Review

For PR-by-PR verification, create a fresh target from the pinned upstream base:

```bash
git switch -c integration/g1-keyboard-pr-review 5aa74557f84c54d4b458f8b9643c5aa2982acfed
git push -u origin integration/g1-keyboard-pr-review
```

Open internal PRs against that target in plan order 0-8. Merge with merge commits,
fetch/pull after each merge, and run the cumulative suites in the
[command guide](branch-stack-commands.md#exact-test-suites). Do not squash shared
dependency ancestry. The existing combined acceptance branch is not a fresh trial.
Upstream-facing PR bases follow the plan's source-parent column instead.

Run manual keyboard commands on stage 1, repeat G1-29 on stage 2, and review both
native models on stage 3. Then review Cartesian sweeps and live XR/video as described
in the command guide. Keep the Hub torso support enabled; keyboard CLI acceptance
is simulation-only without a whole-body controller. Previous headset reviews of
the old release do not transfer to this revision.

## Historical Evidence

- [September 18 simulation-before-Cartesian report](verification/lerobot-branch-stack-20260918.json).
- [September 17 bugfix update](verification/lerobot-branch-stack-20260917-bugfix-update.json).
- [September 11 original acceptance](verification/lerobot-branch-stack-20260911.json).

These reports and their integration refs are preserved, not overwritten. The
coworker installer remains pinned to `d5e400bc` until explicit promotion after
manual acceptance. See the [handoff](branch-stack-handoff.md).
