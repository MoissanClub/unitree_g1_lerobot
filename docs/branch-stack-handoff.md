# G1 Contribution Handoff

## Resume Here

The updated stack is committed and pushed to `MoissanClub/lerobot`.
Use `integration/g1-acceptance-keyboard-20260918`. Its exact tested tip is recorded in
the [verification report](verification/lerobot-branch-stack-keyboard-20260918.json).

- Contribution checkout: `../lerobot-upstream`.
- Fresh verified review checkout: `../lerobot-acceptance-keyboard-20260918-final`.
- Upstream base: `5aa74557f84c54d4b458f8b9643c5aa2982acfed`.
- Source dependencies and PR order: [branch plan](lerobot_g1_branching_refactor_plan.md).
- Commands, tested tips and evidence: [verification guide](branch-stack-verification.md).
- Copy-paste commands for each branch: [acceptance commands](branch-stack-commands.md).

Paths are relative to this repository. Preserve the original `../lerobot`
editable checkout and this repository's experimental DDS/XR runtime.

## Verified State

All nine branch-local suites and nine sequential merges passed without merge
conflicts. The final merge passed **407 targeted tests**, no skips,
and one separate offscreen GPU test. Both embodiments' headless examples passed.
No X window, live headset or physical device was used for this revision.

The stack now has bugfix-first and keyboard-first branches, action-processor IK, upstream-compatible
Hub configuration and optional hands under the standard `UnitreeG1` identity.
No GitHub contribution PRs have been opened or merged.

The user's `e272f385` SDK serialization regression remains in every descendant.
Keyboard jogging first uses the existing G1-29 Hub simulator through the standard
CLI, with no local native backend or Cartesian dependency. Embodiments then select
the keyboard layout automatically from the robot configuration. Simulation depends
on embodiments; Cartesian follows simulation, then
XR and video. Hands branch from simulation independently. Runtime joint simulation
does not construct IK and uses MuJoCo gravity compensation. The existing
`lerobot-teleoperate` entry point supports native viewing with explicit model paths.
Read the [keyboard-first migration note](branch-stack-update-keyboard-20260918.md) before
updating local branches: old tips are archived, not merged into the reordered stack.

## Release Versus Review

The coworker installer still pins `d5e400bcefeccc93ba956ce876530e5283512df1`,
the earlier operator release. Its installation and three-process launcher are
documented in the [coworker guide](coworker-installation.md).

The new contribution stack is a separate review candidate. Previous headset
reviews do not establish acceptance of its changed code. Do not move the
installer pin simply because the newer branch passed automated tests.

## Next Actions

1. Review the branch diffs against the source parents in the plan.
2. Run the [manual keyboard/X/headset checks](branch-stack-commands.md)
   on the new cumulative commit for G1-29 and G1-23. Record the commit, setup and
   observations, including independent arms, release/re-engagement and camera recovery.
3. After manual acceptance, explicitly decide whether to promote the operator pin.
4. For a PR-by-PR trial, start from the current pinned base and merge all nine
   branches in order, beginning with `g1/bugfixes`. Rerun the cumulative suites
   after every merge; the already-combined branch is not that trial's target.
5. Resolve packaging/API/provenance review before upstream submission. Plan physical
   validation separately; G1-23 hardware and BrainCo simulation remain gated.

## Boundaries

Native simulation is fixed-base, arm-only and collision-free. The Hub model path
and the experimental multiprocess DDS path are distinct. Offscreen video tests
do not prove live CloudXR delivery, and fake/installed SDK contract tests do not
prove physical motion or stop behavior. Broader experimental work remains in the
[non-physical backlog](future-non-physical-work.md).

Earlier notes are in the [archive](archive/README.md); the
[Rung 4 handoff](rung4-handoff.md) is experimental history, not this stack's resume point.
