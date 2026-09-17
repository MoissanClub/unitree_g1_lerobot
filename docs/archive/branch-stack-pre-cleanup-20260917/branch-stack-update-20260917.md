# Superseded Snapshot: branch-stack-update-20260917.md

Archived before documentation consolidation on 2026-09-17 from commit
`2589816`. This is historical material, not current execution guidance.
Relative document links below are adjusted for the archive location; commands
and status statements are preserved as they were.

Use the [current branch plan](../../lerobot_g1_branching_refactor_plan.md),
[verification guide](../../branch-stack-verification.md), and
[handoff](../../branch-stack-handoff.md) instead.

---

# G1 Contribution Stack Update: 2026-09-17

## Scope

Upstream base: `5aa74557f84c54d4b458f8b9643c5aa2982acfed`.
Contribution source: `../lerobot-upstream`. Local branch updates preserve the
existing remote histories and the user's September 15 merges. No original
experimental source or installed coworker runtime is replaced.

The standalone `g1/bugfixes` branch corrects the SDK motor velocity field from
`qd` to `dq`, with a stale-velocity regression. It precedes `g1/embodiments`.

## Architecture Changes

- `UnitreeG1` remains one robot identity selected by embodiment and execution mode.
- Upstream `G1EndEffector` and `UnitreeG1MujocoEnv` configuration are preserved;
  the Hub connection receives the typed configuration, not just its repository name.
- `G1CartesianActionProcessor` is registered as `g1_cartesian_to_joints`. It
  consumes root-frame `left.ee_pose` / `right.ee_pose` matrices and measured joint
  observations, calls the existing bounded solver, and emits named `.q` radians.
  These pose matrices are intermediate actions, not a new flat dataset schema.
- `G1XRControl` uses that processor. Tracking, timestamps and independent clutches
  remain in the XR adapter; numerical IK remains independent of the input device.
- `UnitreeG1Config.hands` composes optional hand drivers without a second body
  publisher. Namespaced features, validation before writes, failure cleanup and
  body startup before hand connection are tested. `G1WithHands` remains a legacy
  compatibility adapter, not the recommended new API.
- BrainCo uses `end_effector="brainco"` with explicit hand configurations, ports
  and hardware opt-in. Simulation rejects physical hand drivers and rejects an
  unimplemented BrainCo model before opening a transport.
- The native fixed-base diagnostic simulator defaults to `dummy`; an explicit
  `dex1`/`dex3` request is rejected instead of implying articulated finger physics.
  The upstream Hub path retains its `dex1` default and camera/viewer constraints.

## Verification

The updated `tools/verify_lerobot_branch_stack.py` checks all eight branch tips
and fresh sequential merges from the recorded base. It also runs both embodiments'
headless examples, SDK pipeline construction, camera typing, lint/format and
optional offscreen GPU pixel checks. No X, headset or physical device is opened.

**Passed:** all eight independent branch suites and all eight sequential merge
suites, with no merge conflicts. Final branch: `integration/g1-acceptance-20260917`
at `9936da569a19707c13b4d3de4f8f178dd1012ad9`, available in both the contribution
checkout and `../lerobot-acceptance-20260917-v4`.

| Branch | Verified tip | Branch passes | Cumulative passes |
|---|---|---:|---:|
| `g1/bugfixes` | `f2579395` | 95 | 95 |
| `g1/embodiments` | `3e7a7167` | 131 | 131 |
| `g1/cartesian-control` | `157611d1` | 187 | 187 |
| `g1/simulation` | `097b6872` | 202 | 202 |
| `g1/xr` | `a4621adf` | 212 | 229 |
| `g1/xr-video` | `9739dd17` | 223 | 242 |
| `g1/hand-support` | `571854d6` | 217 | 257 |
| `g1/brainco-hands` | `2d9d762b` | 247 | 287 |

Each non-bugfix stage has one optional SONIC module skip. The video branch and
cumulative stages 5-7 each also passed a separate offscreen GPU pixel test.
Ruff lint/format, camera-channel mypy, installed SDK contract checks and both
embodiments' headless examples passed. The standard teleoperate CLI help and
G1-23 diagnostic configuration parsing were also checked without opening devices.
These are targeted regressions, not the entire upstream test tree.

[Recorded commands and results](../../verification/lerobot-branch-stack-20260917.json).
Full command logs and JUnit XML are in `outputs/branch-stack-20260917-v4/` locally.
Earlier attempts remain available: v1 caught formatting, v2 identified sandbox
GPU visibility, and v3 caught another formatting issue. The successful v4 run
used approved GPU access, remained headless and opened no hardware transport.

**Publication:** all eight contribution branches, updated fork `main`, and
`integration/g1-acceptance-20260917` are pushed to `MoissanClub/lerobot`.
Documentation and verification evidence are published in `MoissanClub/unitree_g1_lerobot`.
No branches were force-rewritten, and no GitHub PR was opened. The original
`integration/g1-acceptance` remote and installer commit pin remain unchanged.

```bash
python tools/verify_lerobot_branch_stack.py \
  --remote git@github.com:MoissanClub/lerobot.git \
  --checkout ../lerobot-review-updated \
  --output outputs/branch-review-updated \
  --assets ../.cache/g1-cartesian-assets \
  --python ../.venvs/lerobot-upstream/bin/python \
  --sdk-site-packages /home/dwei/.venvs/isaacteleop/lib/python3.12/site-packages \
  --gpu --integration-branch integration/g1-acceptance-20260917
```

Use a new checkout/output path for each run. The recorded verification cloned
the local contribution repository; its exact tested branch tips are now published
on origin. The dependency environment is reused; this is not a clean-OS installer test.

## Remaining Boundaries

This revision aligns interfaces; it does not claim the full eight-combination
simulation/hardware/hand support matrix is physically implemented or certified.
G1-23 physical connection remains gated. BrainCo articulated simulation, hand
mass/payload calibration, anatomical motor verification, and physical stop behavior
remain separate work. The fixed-base diagnostic simulator is not a whole-body,
contact-rich or locomotion simulator. The external Hub model repository still
needs G1-23/BrainCo model work before those combinations are available there.

The generic XR reader already follows the merged Isaac example's SDK pattern;
this update does not relocate or replace upstream's SO-101 example. Shared adapter
placement remains a maintainer discussion. No new CloudXR runtime is introduced.

The installed operator pin and the previously headset-accepted branch are preserved.
Review X/headset behavior on the new cumulative branch before changing that pin.
