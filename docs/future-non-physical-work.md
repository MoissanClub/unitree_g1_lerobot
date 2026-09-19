# Non-Physical Acceptance Backlog

This is the technical simulation/reliability backlog, not an alternative branch plan.
[Execution milestones](project-plan.md) determine product priorities and
[development policy](development-model.md) determines submission order.
No physical actuation is authorized by these diagnostics.

## Preserve the Baseline

Both embodiments already have mapping/IK/motor settings, supported-arm simulation,
DDS feedback, bilateral XR, and historical user-reviewed camera delivery.
Small-signal live acceptance recorded G1-29 46/46 cases and G1-23 38/38 cases.
The different case counts reflect supported degrees of freedom, not automatically
inferior acceptance. See [live results](live-control-acceptance.md) and
[independent geometry](independent-geometry.md).

The diagnostic organization migration is complete; do not repeat camera bring-up,
folder migration or the original small-signal suite as if unimplemented.
That baseline does not establish Hub parity, exhaustive headset recovery, whole-body
control or a new fork revision's acceptance.

## Work Items

| Work | Required outcome |
|---|---|
| Broader motion | Larger Cartesian sweeps, conservative limit approaches, holds/reversals, unilateral and bilateral motion; separate reachable targets from workspace characterization |
| Failure/recovery | Independent clutch release, invalid/lost tracking, stale input, session/camera/simulator restart and reconnect; verify no stale replay or unexpected re-engagement |
| Performance/endurance | Matched trajectories with video off/on; physics real-time factor, loop jitter, tracking, frame age/drops and resource growth with declared duration/budgets |
| Native lifecycle | SDK-only DDS teardown reproducer and explicit-cleanup comparison; investigate OpenXR shutdown independently |
| Matched models | Before waist control, match G1-29 URDF/MJCF families and reverify full-chain FK, gravity/inertia, limits and meshes |
| Hub parity | P2 in the execution plan: reuse public robot APIs and verify both embodiments without relabeling native-backend results |

Report target, commanded and measured poses; distinguish IK residual, actuator error,
inactive-arm drift and limit margins. Set per-case thresholds before running.
G1-23 five-joint arms cannot generally achieve arbitrary six-dimensional hand poses.
Preserve independent pelvis-frame checks and LeRobot-default G1-29 versus derived
G1-23 runtime gains. Derived G1-29 gains remain a separate benchmark, not an unnoticed
runtime change. Live command-pose feedforward and exact-model measured-pose benchmark
compensation are not interchangeable evidence.

DDS callback teardown responsibility remains unresolved among lifecycle/SDK/bindings;
fresh-process tests contain the issue, not fix it. Do not describe it as a confirmed
CycloneDDS transport defect. Bring lifecycle work forward if it blocks fault testing.

Camera frame age is not end-to-end headset latency. Test actual headset presentation
and reconnection manually where automated observation is insufficient. Keep one command
publisher; camera loss and control loss must have explicitly separate behavior.

## Implementation and Evidence

Reuse `unitree_g1_lerobot/diagnostics/shared/` cases, metrics and reports;
`backends/simulation.py` owns session execution. Extend
`diagnostics/verify_live_control.py` and `tests/diagnostics/` as needed.
Do not create separate simulation/physical copies of trajectory or metric logic.

Deliver a selectable broader suite through `run_verify_live_control.sh`, preserve
the original suite, and retain per-case evidence for both embodiments. Do not change
models, gains, IK objectives or thresholds merely to improve the pass count.
Workspace tests do not establish collision safety, maximum hardware speed or payload.

Record revisions, configuration, source hashes, commands and numerical reports.
Keep historical JSON unchanged; compact reports live in `docs/verification/`, with
larger machine-local logs linked explicitly. Physical work is separately gated by
[sim-to-real acceptance](sim-to-real-plan.md).
