# Branch Plan: Current Location

The old stacked-branch development strategy has been superseded.

Latest G1 VR candidate: `work/g1-vr-teleoperate` uses the standard
`lerobot-teleoperate` loop. `work/teleoperate-processors` isolates its generic CLI
change on the bugfix baseline. The standalone `work/g1-29-vr-teleop` checkpoint
`2b9998b55` is preserved, not overwritten. See the development model below and
`docs/source/g1_vr_standard_cli.mdx` on the new LeRobot branch.

- [Development model](development-model.md): active branches, work/submission flow,
  upstream reconciliation and release policy.
- [Execution plan](project-plan.md#small-upstream-contributions): proposed PR slices,
  actual prerequisites and verification gates.
- [Reference migration](branch-reference-migration-20260918.md): exact archived tips.
- [Historical commands](branch-stack-commands.md): reproduce the frozen nine-stage
  series; these are not new upstream PR dependencies.

The [previous branching plan](archive/planning-checkpoint-20260918/lerobot_g1_branching_refactor_plan.md)
is retained only for provenance.
