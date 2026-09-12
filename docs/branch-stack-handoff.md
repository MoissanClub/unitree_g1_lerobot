# LeRobot Branch Reorganization Handoff

## Verified Checkpoint: 2026-09-11

All seven remote branches passed branch-local regression and conflict-free
sequential merges in a fresh checkout. Final merged result: **275 tests passed,
1 optional SONIC skip, plus 1 separate offscreen GPU test passed**. Lint, format,
camera-channel typing, and both embodiments' headless examples passed.
The verified merge commit is `d5e400bcefeccc93ba956ce876530e5283512df1`.
Evidence is linked from the verification guide. Fork main is unchanged.

## Scope

The seven feature branches are implemented and pushed to `MoissanClub/lerobot`:
embodiments, Cartesian control, simulation, XR, XR-video, generic hand composition,
and BrainCo Revo2. Source checkout: `../lerobot-upstream`. The fresh review checkout
is `../lerobot-g1-review`; its cumulative branch is `integration/g1-acceptance`.
See [verification and reproduction](branch-stack-verification.md) and the
[branch plan](lerobot_g1_branching_refactor_plan.md).

The original `../lerobot` user changes and this project's working DDS/XR scripts
were preserved. This project's runtime package, configurations, and existing tests
were also preserved; documentation, the new `tools/verify_lerobot_branch_stack.py`,
and verification evidence changed (checkpoint `4a47665`). Preservation refers to
the working runtime, not an entirely untouched repository. No physical robot/hand
was contacted. After the user's headless-only
instruction, no X viewer or headset session was launched. Offscreen GPU tests are
not headset acceptance. No GitHub PRs were created or merged by the runner; it
reproduces the merge sequence locally with ordinary merge commits.

## Next Session

1. Review the branch diffs and recorded fresh-checkout evidence. Branches keep
   their declared source parents; XR/video have no artificial simulation ancestry.
2. Run the X and headset commands in the verification guide on the merged fork.
   Review native G1-29/G1-23 geometry, scripted/keyboard motion, independent clutch
   control, rotations, release/re-engage, tracking loss, and camera delivery.
3. For your own PR-by-PR trial, create another integration branch from the recorded
   upstream base. Merge features in milestone order without squashing shared
   ancestry, and use each milestone's cumulative suites from the runner.
4. Before hardware work, confirm BrainCo six-slot anatomical mapping, model/side,
   serial IDs, software limits, tactile units, and stop behavior. Hardware access
   defaults off; SDK tests inspect symbols and fake responses only.
5. Resume broader experimental motion/recovery/endurance work separately. The new
   fork's native arm simulator does not replace the existing multiprocess DDS
   workflow or resolve its native teardown issue.

## Important Boundaries

- LeRobot G1-29 gain defaults and its old runtime are preserved.
- The native simulator uses the explicit embodiment URDF, fixed non-arm joints,
  collision-free arm dynamics, configured PD, and optional measured-pose gravity
  feedforward. Example launchers enable gravity compensation by default.
- XR input is generic; G1 retargeting/IK belongs to the robot adapter.
- Video uses a same-host bounded RGB channel and an isolated input/display worker
  sharing one OpenXR session. The combined example renders the simulator in its
  local loop and is not a performance-tuned DDS deployment.
- `G1WithHands` wraps the body rather than adding SDK dependencies to `UnitreeG1`.
  BrainCo targets the audited 2.0.2 wheel; it uses module-level serial close.
- Isaac Teleop's partial-startup cleanup required a guarded private ExitStack
  workaround. Native failures inside SDK context entry remain vendor-owned.

The earlier end-of-day notes in `rung4-handoff.md` remain experiment history, not
the current branch-reorganization resume point.
