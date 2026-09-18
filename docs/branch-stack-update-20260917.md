# G1 Stack Revision: 2026-09-17

This dated note records what changed, not a second execution plan.
Use the [branch plan](lerobot_g1_branching_refactor_plan.md) for dependencies,
[verification guide](branch-stack-verification.md) for commands/results, and
[handoff](branch-stack-handoff.md) for the next session.

## Changes

- Updated the contribution stack to upstream `5aa74557`, including merged
  end-effector/camera configuration from PR #4645.
- Extracted the SDK `qd` -> `dq` correction into `g1/bugfixes` before embodiments.
- Added registered Cartesian action-processor IK and routed G1 XR targets through it.
- Preserved typed Hub configuration and made the native diagnostic simulator's
  bare-wrist-only capability explicit.
- Added optional-hand composition through `UnitreeG1Config.hands`; the prior
  `G1WithHands` wrapper remains for compatibility.
- Aligned BrainCo with `end_effector="brainco"`, explicit physical configurations
  and rejection of unimplemented simulation.
- Made hand support depend on simulation to resolve shared robot-lifecycle changes.
  XR/video remain simulation-independent in source ancestry.
- Preserved remote histories, existing operator scripts and the installer pin.
  No force push or GitHub PR was part of this update.

## Evidence And Publication

All eight branches and the cumulative `integration/g1-acceptance-20260917` branch
were pushed. The [recorded report](verification/lerobot-branch-stack-20260917.json)
contains exact tips, commands and results from a fresh local clone; those tested
tips were then published. Initial cumulative commit: `9936da56`.

Full logs and JUnit XML remain in `outputs/branch-stack-20260917-v4/` locally.
Earlier attempts found formatting issues and sandbox GPU visibility restrictions;
the successful run used approved GPU access without opening X or hardware.
Manual acceptance and unsupported combinations are listed in the current plan.

## Serialization Regression Follow-Up

The user's `g1/bugfixes` update `e272f385` replaces the mocked velocity check
with two real SDK serialization cases: configured gains/torque and explicit
per-joint overrides. It verifies the wire-format position, velocity, gains,
torque and unchanged neighboring motor command without creating a DDS participant.

Ordinary merges propagated it through all seven feature branches and the combined
branch, now `da1c0fcd`. No production code or installer pin changed. A fresh clone
passed all eight branch-local and eight sequential-merge gates, ending with 320
targeted passes, zero skips and a separate GPU pass. The increased count includes
32 SONIC cases enabled by available ONNX dependencies, plus one extra serialized
command case. Both embodiments' headless launchers also passed.

The [new report](verification/lerobot-branch-stack-20260917-bugfix-update.json)
retains exact tips and commands; full logs are in `outputs/branch-stack-20260917-v6/`.
The published cumulative tree is identical to the fresh sequential merge tree.
The first follow-up run passed all branch-local gates but hit a local trial branch
name collision; the successful rerun used a distinct trial name. The original
report above remains unchanged. X/headset and physical acceptance remain pending.

The [pre-cleanup document snapshots](archive/README.md) preserve the original
reconstruction instructions and mixed historical/current notes.
